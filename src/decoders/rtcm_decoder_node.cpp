#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <sstream>
#include <optional>
#include <string>
#include <map>
#include <array>
#include <set>
#include <vector>

#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"


using namespace std::chrono_literals;

namespace gnss_ros_standardization {

namespace {
constexpr uint8_t  RTCM3_PREAMBLE = 0xD3;
constexpr uint16_t RTCM3_MAX_LEN  = 1023;
constexpr size_t   RTCM3_HDR_LEN  = 3;     // preamble + length(2)
constexpr size_t   RTCM3_CRC_LEN  = 3;
constexpr uint32_t CRC24Q_POLY    = 0x1864CFB; // CRC-24Q

// Relay-output carry cap: about one minute of corrections at typical RTCM
// rates (2-5 KB/s). Overflow means the relay target has been down for minutes
// or the output link is persistently too slow; corrections that old are
// rejected by receivers via age-of-differential anyway, so the whole carry is
// dropped with a WARN (still gentler than STRSVR's immediate silent drop).
constexpr size_t RELAY_CARRY_MAX  = 256 * 1024;
} // namespace

/// @brief ROS 2 node for decoding RTCM3 messages from NTRIP/serial/file streams
///
/// Reads RTCM3 frames (preamble 0xD3 + length + payload + CRC-24Q) from the
/// configured stream and publishes observation epochs (/gnss/observation) and
/// ephemerides (/gnss/ephemeris) via RTKLIB's input_rtcm3() decoder.
class RtcmDecoderNode : public rclcpp::Node {
public:
  RtcmDecoderNode() : Node("rtcm_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    openRtcmRelay();
    startPolling();
    RCLCPP_INFO(get_logger(), "RTCM Decoder started");
  }

  ~RtcmDecoderNode() override {
    try {
      strclose(&stream_);
      if (relay_enabled_) strclose(&relay_out_);
    } catch (...) {}
    free_rtcm(&rtcm_);
  }

private:
  // Initialization

  wire::ReceiverHealth receiver_health_;
  void initializeParameters() {
    receiver_health_.configure(5.0);
    receiver_health_.limits(MAXOBS, NFREQ, NEXOBS, MAXPRNCMP);
    declare_parameter<std::string>("stream_path", "");
    declare_parameter<bool>("use_gps_timestamp", false);
    for (const auto& system : declare_parameter<std::vector<std::string>>(
             "observation.excluded_systems", std::vector<std::string>{})) {
      const auto lane=std::string("GRESJCI").find(system);
      if (system.size()!=1 || lane==std::string::npos)
        throw std::invalid_argument("observation.excluded_systems requires G R E C J I S");
      excluded_lanes_ |= 1u<<lane;
    }
    declare_parameter<int>("reference_week", 0);
    declare_parameter<double>("reference_tow", 0.0);
    declare_parameter<int>("station_id", -1);  // -1: accept station changes; nonnegative: explicit filter
    station_id_ = get_parameter("station_id").as_int();
    if (station_id_ < -1 || station_id_ > 4095) throw std::invalid_argument("station_id must be -1 or 0..4095");
    declare_parameter<std::string>("topics.observation", "/gnss/observation");
    declare_parameter<std::string>("topics.ephemeris", "/gnss/ephemeris");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all

    // RTCM relay output (opt-in, see openRtcmRelay). Empty = disabled.
    declare_parameter<std::string>("rtcm_output.destination", "");

    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));

    if (get_parameter("stream_path").as_string().empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'stream_path' is required but not set. "
        "Example: --ros-args -p stream_path:=\"ntrip://user:pass@host:port/mountpoint\"");
      throw std::runtime_error("stream_path parameter is required");
    }
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<gnss_ros_standardization::msg::GnssObservations>(
        get_parameter("topics.observation").as_string(), 10);
    nav_pub_ = create_publisher<gnss_ros_standardization::msg::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(),
        rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
  }

  void initializeDecoder() {
    if (init_rtcm(&rtcm_) != 1) {
      RCLCPP_ERROR(get_logger(), "init_rtcm failed");
      throw std::runtime_error("init_rtcm failed");
    }
    std::strcpy(rtcm_.opt, "-EPHALL");
    const int week = get_parameter("reference_week").as_int();
    const double tow = get_parameter("reference_tow").as_double();
    if (week < 0 || !std::isfinite(tow) || tow < 0 || tow >= 604800)
      throw std::invalid_argument("Invalid RTCM reference_week/reference_tow");
    have_absolute_date_=week>0;
    if (week > 0) rtcm_.time = gpst2time(week, tow);
    else
      RCLCPP_WARN(get_logger(), "RTCM without reference_week/reference_tow uses the host date until a valid 1013/1020 absolute date arrives. Historical replay without an in-band date needs an explicit reference; ambiguous navigation may be withheld");
  }

  void acceptAbsoluteDate(gtime_t reference, int type) {
    // Retain the latest pre-reference GPS frame per satellite, bounded by the
    // decoder's satellite range. Replay it after dating is known, not with
    // invented toe/ttr values copied from the host-date decode.
    auto pending=std::move(unreferenced_gps_);
    if (rtcm_.time.time && std::abs(timediff(reference,rtcm_.time))>302400) {
      resetReceiverSession(false,reference);
    } else rtcm_.time=reference;
    have_absolute_date_=true;
    unreferenced_gps_.clear();
    for (const auto& entry:pending) {
      int result=0;
      for (auto byte:entry.second) result=input_rtcm3(&rtcm_,byte);
      if (result==2) {
        resolveGpsNavigationWeek(1019);
        publishEphemeridesIfChanged(true);
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"RTCM date reference supplied by message %d",type);
  }

  // RTCM 1013 carries an absolute MJD/UTC day and GPS-UTC offset. The pinned
  // backend decodes these into a local variable but does not seed rtcm.time.
  // Use a complete valid announcement when no explicit reference was supplied.
  void acceptTimeAnnouncement(const uint8_t* frame, size_t payload_size) {
    if (get_parameter("reference_week").as_int()>0 || payload_size<9) return;
    const int mjd=getbitu(frame,48,16), sod=getbitu(frame,64,17);
    const int count=getbitu(frame,81,5), leaps=getbitu(frame,86,8);
    if (payload_size*8<70u+29u*count || mjd<44244 || sod>=86400 || leaps==255) return;
    const double ep[]={1980,1,6,0,0,0};
    const auto reference=timeadd(epoch2time(ep),double(mjd-44244)*86400+sod+leaps);
    acceptAbsoluteDate(reference,1013);
  }

  void acceptGlonassDate(const uint8_t* frame, size_t payload_size) {
    // R2 msg1020 + GLONASS ICD 5.1: NT is the 1-based calendar day in
    // the four-year interval N4 (1 starts in 1996). tk is UTC(SU)+3 h.
    // Use this only to bootstrap dating; later, older ephemerides must not
    // rewind the stream's measurement time. Explicit references always win.
    if (have_absolute_date_ || payload_size<45 || !getbitu(frame,24+281,1)) return;
    const int nt=getbitu(frame,24+268,11), n4=getbitu(frame,24+325,5);
    const int hour=getbitu(frame,24+27,5), minute=getbitu(frame,24+32,6);
    const int sat=getbitu(frame,24+12,6), fcn=getbitu(frame,24+18,5);
    if (nt<1 || nt>1461 || n4<1 || n4>26 || hour>23 || minute>59 ||
        sat<1 || sat>MAXPRNGLO || fcn>13) return;
    const double ep[]={1996.0+4*(n4-1),1,1,0,0,0};
    const auto utc=timeadd(epoch2time(ep),(nt-1)*86400.0+hour*3600+minute*60+
                           getbitu(frame,24+38,1)*30-10800);
    acceptAbsoluteDate(utc2gpst(utc),1020);
  }

  void resolveGpsNavigationWeek(int type) {
    if (type!=1019 || rtcm_.ephsat<=0) return;
    auto& e=rtcm_.nav.eph[rtcm_.ephsat-1];
    int reference_week=0;
    time2gpst(rtcm_.time,&reference_week);
    const int wire_week=getbitu(rtcm_.buff,42,10);
    int week=wire_week+1024*static_cast<int>(std::llround((reference_week-wire_week)/1024.0));
    // Resolve the truncated week using this stream, never RTKLIB's global
    // host clock. Keep the usual adjacent-week toe adjustment at a real week
    // boundary, but do not shift a historical week toward an unrelated date.
    const double toc=time2gpst(e.toc,nullptr);
    const double delta=timediff(gpst2time(week,e.toes),rtcm_.time);
    if (std::abs(delta)<604800) {
      if (delta < -302400) ++week;
      else if (delta >= 302400) --week;
    }
    e.week=week;
    e.toe=gpst2time(week,e.toes);
    e.toc=gpst2time(week,toc);
    const double clock_delta=timediff(e.toc,e.toe);
    if (clock_delta < -302400) e.toc=timeadd(e.toc,604800);
    else if (clock_delta > 302400) e.toc=timeadd(e.toc,-604800);
  }

  void startPolling() {
    timer_ = create_wall_timer(10ms, std::bind(&RtcmDecoderNode::onTimer, this));
  }

  static uint32_t crc24q(const uint8_t* p, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
      crc ^= (uint32_t)p[i] << 16;
      for (int b = 0; b < 8; ++b) {
        crc <<= 1;
        if (crc & 0x1000000) crc ^= CRC24Q_POLY;
      }
      crc &= 0xFFFFFF;
    }
    return crc;
  }

  // ---- stream --------------------------------------------------------------

  void openStream() {
    if (!decoder_common::openStreamPath(get_logger(), stream_,
                                        get_parameter("stream_path").as_string(),
                                        STR_MODE_R, "Input")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // RTCM relay output (RTKLIB STRSVR-style): when `rtcm_output.destination` is set (e.g.
  // "tcpcli://127.0.0.1:5556"), forward raw source bytes verbatim to that
  // stream — typically the TCP server hosted by a receiver driver/decoder node
  // (ubx=5556 / sbf=5557 / novatel=5558), which writes them to the receiver
  // for on-chip RTK. A spare receiver port can also be fed directly
  // (e.g. "serial:///dev/ttyUSB2:230400"). Decoding to topics is unaffected.
  void openRtcmRelay() {
    const auto target = get_parameter("rtcm_output.destination").as_string();
    if (target.empty()) return;
    if (!decoder_common::openStreamPath(get_logger(), relay_out_, target,
                                        STR_MODE_RW, "RTCM relay")) {
      throw std::runtime_error("RTCM relay stropen failed");
    }
    relay_enabled_ = true;
    RCLCPP_INFO(get_logger(), "RTCM relay enabled: source -> '%s'", decoder_common::redactStreamUri(target).c_str());
  }

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timelines_[7];
  int active_station_ = -1;
  void discardMsmAssembly(bool count=true) {
    if (count) for (const auto& [key,epoch]:pending_msm_epochs_)
      receiver_health_.msmIncompleteDiscarded(epoch.fragments);
    pending_msm_epochs_.clear();
    pending_msm_times_ = {};
    msm_iod_ = -1;
  }
  void resetReceiverSession(bool disconnected, gtime_t reference={}) {
    const bool dated=have_absolute_date_;
    last_published_.reset(); msm_iod_=-1; discard_until_complete_=false;
    discardMsmAssembly();
    free_rtcm(&rtcm_); rtcm_={}; initializeDecoder();
    unreferenced_gps_.clear();
    if (!disconnected && reference.time) { rtcm_.time=reference; have_absolute_date_=dated; }
    if (disconnected) { rx_.clear(); rx_off_=0; }
    station_id_=get_parameter("station_id").as_int();
    for (auto& timeline : receiver_timelines_) timeline.reset();
    active_station_ = -1; eph_store_.clear();
    receiver_health_.resetSession();
    nav_pub_->publish(eph_store_.buildSnapshot(now()));
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RTCM session reset: %s; incomplete epoch and station/time/navigation caches cleared",
                disconnected ? "connection lost" : "measurement rewind");
  }

  void onTimer() {
    uint8_t buf[4096];
    const int n = strread(&stream_, buf, sizeof(buf));
    receiver_health_.input(n);
    receiver_health_.assemblyPending(!pending_msm_epochs_.empty() || (rtcm_.obs.n>0 && !rtcm_.obsflag));
    receiver_health_.report(get_logger());
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    // STRSVR-style relay: forward the raw bytes verbatim toward the receiver
    // driver before (and independent of) local decoding. Runs on idle reads
    // too, so a pending carry keeps draining once the relay target recovers.
    // Unlike the receiver-side relay, source reads cannot be paused for
    // backpressure (that would stall topic decoding and NTRIP casters drop
    // slow clients), so the CarriedStreamWriter is capped instead.
    if (relay_enabled_) {
      const size_t before = relay_writer_.bytes;
      relay_writer_.write(get_logger(), *get_clock(), relay_out_,
                          n > 0 ? buf : nullptr, n > 0 ? n : 0);
      if (relay_writer_.bytes > before) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "RTCM relay: %zu bytes forwarded (total)",
                             relay_writer_.bytes);
      }
    }
    if (n > 0) {
      last_data_ = std::chrono::steady_clock::now();
      rx_.insert(rx_.end(), buf, buf + n);
      decodeRtcm3FromBuffer();
    }
    if (n <= 0 && std::chrono::steady_clock::now()-last_data_ > 1s)
      decodeRtcm3FromBuffer(true);
    // RTKLIB's file status distinguishes EOF from an idle live connection.
    // Neither EOF nor silence may promote an unfinished chain to an epoch.
    if (n<=0 && stream_.type==STR_FILE) {
      char status[MAXSTRMSG]{}; strstat(&stream_,status);
      if (std::strcmp(status,"end")==0) {
        decodeRtcm3FromBuffer(true);
        discardMsmAssembly(); rtcm_.obs.n=0; rtcm_.obsflag=1;
      }
    }
    publishEphemeridesIfChanged(false);
  }

  // ---- RTCM3 framing & decode ---------------------------------------------
  //
  // Read-offset + amortized compact strategy: rx_off_ tracks the read head into
  // rx_. Resync (preamble miss / CRC fail) is O(1) (just ++rx_off_), and erase
  // only fires when the unread tail exceeds half of rx_, yielding amortized O(n)
  // even on noisy streams (vs. O(n²) of per-byte erase(begin())).
  void decodeRtcm3FromBuffer(bool expire = false) {
    while (true) {
      const size_t avail = rx_.size() - rx_off_;
      if (avail < RTCM3_HDR_LEN) break;
      const uint8_t* head = rx_.data() + rx_off_;
      if (head[0] != RTCM3_PREAMBLE) { ++rx_off_; continue; }

      const uint16_t plen = ((head[1] & 0x03) << 8) | head[2];
      if ((head[1] & 0xfc) || plen > RTCM3_MAX_LEN) { ++rx_off_; continue; }

      const size_t flen = RTCM3_HDR_LEN + plen + RTCM3_CRC_LEN;
      if (avail < flen) {
        if (!expire) break;
        ++rx_off_; continue;  // abandoned truncated frame; recover a trailing frame
      }

      const uint32_t crc_calc = crc24q(head, flen - RTCM3_CRC_LEN);
      const uint32_t crc_recv =
          (uint32_t(head[flen - 3]) << 16) |
          (uint32_t(head[flen - 2]) << 8)  |
           uint32_t(head[flen - 1]);

      if (crc_calc != crc_recv) { receiver_health_.rejectFrame(); ++rx_off_; continue; }

      if (plen < 2) {
        rx_off_ += flen; continue;
      }
      const int type = (head[3] << 4) | (head[4] >> 4);
      receiver_health_.frame(type);
      receiver_health_.inspectRtcm(type);
      const bool has_station = (type >= 1001 && type <= 1013) || type == 1033 || type == 1230 ||
          (type >= 1071 && type <= 1137 && type % 10 >= 1 && type % 10 <= 7);
      const int station = has_station && plen>=3 ? ((head[4]&15)<<8)|head[5] : -1;
      if (station >= 0) {
        if (station_id_ >= 0 && station != station_id_) {
          receiver_health_.stationFiltered();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Ignoring RTCM station %d: locked to %d", station, station_id_);
          rx_off_ += flen; continue;
        }
      }
      if (station >= 0 && active_station_ >= 0 && station != active_station_) {
        // A VRS may change station without reconnecting. Keep satellite
        // navigation, but never join measurements or lock history across bases.
        last_published_.reset(); msm_iod_=-1; discard_until_complete_=false;
        discardMsmAssembly();
        rtcm_.obs.n=0; rtcm_.obsflag=1;
        // decode_msm_head clears obsflag before checking staid, so explicitly
        // release the previous station as well as the unfinished observation.
        rtcm_.staid=0;
        std::memset(rtcm_.cp,0,sizeof(rtcm_.cp));
        std::memset(rtcm_.lock,0,sizeof(rtcm_.lock));
        std::memset(rtcm_.loss,0,sizeof(rtcm_.loss));
        std::memset(rtcm_.lltime,0,sizeof(rtcm_.lltime));
        for (auto& timeline : receiver_timelines_) timeline.reset();
      }
      // Feed the CRC-validated frame byte-by-byte; process each completion
      // inline. Standard RTCM3 yields one ret>0 per frame, but the inline
      // check guarantees that any middle-of-frame completion (variant MSM /
      // future spec extensions) is not silently dropped.
      const bool msm = type>=1071 && type<=1137 && type%10>=1 && type%10<=7;
      const bool legacy_obs = type==1002 || type==1004 || type==1010 || type==1012;
      const bool observation = msm || legacy_obs;
      const int lane = msm ? type/10-107 : (type==1010 || type==1012 ? 1 : 0);
      if (msm && plen>=10) {
        const int iod=getbitu(head,79,3);
        if (msm_iod_>=0 && iod!=msm_iod_ && !pending_msm_epochs_.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Discarding incomplete RTCM epoch: MSM issue of data changed");
          rtcm_.obs.n=0; rtcm_.obsflag=1;
          discardMsmAssembly();
        }
        msm_iod_=iod;
      }
      auto diagnostic_scope = receiver_health_.backendScope();
      // These legacy IDs decode headers only in the pinned backend. A header
      // must never complete observations left by an earlier MSM fragment.
      if (type==1001 || type==1003 || type==1009 || type==1011) {
        GNSS_RX_EVENT(GNSS_RX_PAYLOAD_UNSUPPORTED,1);
        receiver_health_.backendResult(-1);
        // R5: an unsupported header cannot complete or erase a valid chain.
        rx_off_ += flen; continue;
      }
      bool nav_changed = false;
      bool rejected = false;
      if (type==1013) acceptTimeAnnouncement(head,plen);
      if (type==1020) acceptGlonassDate(head,plen);
      const auto previous_time=rtcm_.time;
      // Decode each MSM frame independently of RTKLIB's single-time obs_t.
      // Preserve lock/phase history and navigation; assemble validated frames
      // below by their own decoded time until the wire sequence completes.
      if (observation) { rtcm_.obs.n=0; rtcm_.obsflag=0; }
      for (size_t i = 0; i < flen; ++i) {
        const int r = input_rtcm3(&rtcm_, head[i]);
        if (r==2) resolveGpsNavigationWeek(type);
        if (r==2 && type==1019 && !have_absolute_date_)
          unreferenced_gps_[rtcm_.ephsat]=std::vector<uint8_t>(head,head+flen);
        receiver_health_.backendResult(r);
        nav_changed = nav_changed || r == 2;
        rejected = rejected || r < 0;
        // Completion is the backend return value, not an incidental obsflag
        // side effect. In particular, 1002/1010 return 1 on DF005=0 without
        // setting obsflag, unlike 1004/1012 and MSM. Normalize it for the
        // common assembly/split/overflow paths below; rejected frames cannot
        // close a pending chain.
        if (observation && i+1==flen && r>=0) rtcm_.obsflag=(r==1);
        if (r>=0 && i+1==flen && observation && !discard_until_complete_ && rtcm_.obs.n>0) {
          const auto time=rtcm_.time;
          if (!msm || !(excluded_lanes_&(1u<<lane))) {
            const auto key=epochKey(time);
            if (last_published_ && *last_published_==key) {
              // A late fragment is already unusable, even without sync=0.
              // Do not let it vote against the next epoch's valid fragments.
              receiver_health_.observationNotMonotonic();
              if (rtcm_.obsflag) reportSplitCompletion();
              rtcm_.obs.n=0;
            } else {
              if (pending_msm_times_[lane] && !(epochKey(*pending_msm_times_[lane])==key)) {
                // A repeated lane identifies a new chain. Preserve fragments
                // already buffered at that new time, rather than clearing all.
                for (auto it=pending_msm_epochs_.begin();it!=pending_msm_epochs_.end();) {
                  if (!(it->first==key)) {
                    receiver_health_.msmIncompleteDiscarded(it->second.fragments);
                    it=pending_msm_epochs_.erase(it);
                  } else ++it;
                }
                for (auto& t:pending_msm_times_) if (t && !(epochKey(*t)==key)) t.reset();
              }
              pending_msm_times_[lane]=time;
              accumulateObservations(pending_msm_epochs_,time);
              auto& pending=pending_msm_epochs_.at(key);
              pending.lanes |= 1u<<lane; ++pending.fragments;
              size_t count=0;
              for (const auto& item:pending_msm_epochs_) count+=item.second.fragments;
              if (count>256) { discardMsmAssembly(); discard_until_complete_=true; }
            }
          }
        }
      }
      // Pending/rejected measurements never seed navigation dating. Only a
      // chain whose time has been decided can advance the accepted anchor.
      if (!rejected && msm && (excluded_lanes_&(1u<<(type/10-107))))
        receiver_health_.msmSystemExcluded();
      if (observation) rtcm_.time=previous_time;
      if (!rejected && observation && rtcm_.obsflag) {
        if (discard_until_complete_) discard_until_complete_=false;
        else completeMsmChain();
      }
      receiver_health_.assemblyPending(!pending_msm_epochs_.empty() || (rtcm_.obs.n>0 && !rtcm_.obsflag));
      if (rejected) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Rejected RTCM payload: type=%d (invalid or unsupported); dropping frame only",type);
        rtcm_.obs.n=0;
        rtcm_.time=previous_time;
        rtcm_.staid=station_id_>=0 ? station_id_ : 0;
      } else if (station>=0) active_station_=station;
      rx_off_ += flen;
      if (nav_changed) publishEphemeridesIfChanged(true);
    }
    // Amortized compact: only erase when read head has moved past half of rx_.
    if (rx_off_ * 2 > rx_.size()) {
      rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(rx_off_));
      rx_off_ = 0;
    }
  }

  // ---- observations --------------------------------------------------------
  bool discard_until_complete_{false};

  struct EpochKey {
    int week{0};
    int64_t tow_ms{0};
    bool operator==(const EpochKey& o) const { return week==o.week && tow_ms==o.tow_ms; }
    bool operator<(const EpochKey& o) const { return week != o.week ? week < o.week : tow_ms < o.tow_ms; }
  };
  struct EpochKeyHash {
    size_t operator()(const EpochKey& k) const {
      return (size_t(uint32_t(k.week)) << 32) ^ size_t(k.tow_ms);
    }
  };
  struct EpochBuffer {
    std::vector<gnss_ros_standardization::msg::GnssObservation> observations;
    int cnt_G=0,cnt_R=0,cnt_E=0,cnt_J=0,cnt_C=0,cnt_I=0,cnt_S=0,cnt_U=0;
    int    week{0};
    double tow{0.0};
    std::set<int> satellites;
    uint32_t lanes=0;
    size_t fragments=0;
  };

  void mergeObservation(EpochBuffer& epoch, const msg::GnssObservation& obs) {
    const auto existing=std::find_if(epoch.observations.begin(),epoch.observations.end(),
        [&](const auto& previous) { return previous.sat==obs.sat && previous.code==obs.code; });
    if (existing!=epoch.observations.end()) {
      if (!gnss_utils::sameObservation(*existing,obs)) receiver_health_.observationMergeConflict();
      return; // First validated value wins; identical retransmissions are harmless.
    }
    epoch.observations.push_back(obs);
    if (epoch.satellites.insert(obs.sat).second) switch (satsys(obs.sat,nullptr)) {
      case SYS_GPS: ++epoch.cnt_G; break;
      case SYS_GLO: ++epoch.cnt_R; break;
      case SYS_GAL: ++epoch.cnt_E; break;
      case SYS_QZS: ++epoch.cnt_J; break;
      case SYS_CMP: ++epoch.cnt_C; break;
      case SYS_IRN: ++epoch.cnt_I; break;
      case SYS_SBS: ++epoch.cnt_S; break;
      default: ++epoch.cnt_U; break;
    }
  }

  static EpochKey epochKey(gtime_t time) {
    int week=0; const double tow=time2gpst(time,&week);
    return {week,static_cast<int64_t>(std::llround(tow*1000.0))};
  }

  void accumulateObservations(std::map<EpochKey, EpochBuffer>& destination, gtime_t time) {
    const auto key=epochKey(time);
    auto& epoch=destination[key];
    epoch.week=key.week;
    epoch.tow=time2gpst(time,nullptr);
    for (int j=0; j<rtcm_.obs.n; ++j) {
      const auto& o=rtcm_.obs.data[j];
      for (int band=0; band<NFREQ+NEXOBS; ++band) {
        if (o.P[band]==0 && o.L[band]==0 && o.D[band]==0 && o.SNR[band]==0) continue;
        const auto obs=gnss_utils::obsToMsg(o,band,&rtcm_.nav);
        if (obs.sat) mergeObservation(epoch,obs);
      }
    }
  }

  void reportSplitCompletion() {
    receiver_health_.msmSplitDetected();
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
        "RTCM source ends an epoch (multiple message bit 0) before its last observation; later observations at the same time are dropped. Configure the source per RTCM 10403 DF393.");
  }

  void completeMsmChain() {
    if (pending_msm_epochs_.empty()) return;
    auto winner=pending_msm_epochs_.end();
    unsigned best=0;bool tied=false;
    for (auto it=pending_msm_epochs_.begin();it!=pending_msm_epochs_.end();++it) {
      const unsigned votes=__builtin_popcount(it->second.lanes);
      if (votes>best) {best=votes;winner=it;tied=false;}
      else if (votes==best) tied=true;
    }
    if (tied) {
      receiver_health_.msmTimeTieWithheld();
      std::ostringstream detail;detail<<std::fixed<<std::setprecision(3);
      const auto first=pending_msm_epochs_.begin()->first;
      for (const auto& [key,epoch]:pending_msm_epochs_) {
        for (int lane=0;lane<7;++lane) if (epoch.lanes&(1u<<lane)) detail<<"GRESJCI"[lane];
        detail<<" at "<<epoch.tow<<" s (offset "<<std::showpos
              <<(key.week-first.week)*604800.0+(key.tow_ms-first.tow_ms)*.001<<std::noshowpos<<") ";
      }
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
          "MSM time tie: %s; epoch withheld. If a constellation is known faulty, set observation.excluded_systems (e.g. [\"C\"]).",detail.str().c_str());
      discardMsmAssembly(false);return;
    }
    const auto key=winner->first;
    auto epoch=winner->second;
    for (const auto& [other,value]:pending_msm_epochs_)
      if (!(other==key)) for (size_t i=0;i<value.fragments;++i) receiver_health_.msmChainTimeMismatch();
    discardMsmAssembly(false);
    const auto time=gpst2time(epoch.week,epoch.tow);
    bool rewind=false;
    for (int lane=0;lane<7;++lane) if (epoch.lanes&(1u<<lane))
      rewind=receiver_timelines_[lane].observe(double(time.time)+time.sec) || rewind;
    if (rewind) {
      resetReceiverSession(false,time);
      for (int lane=0;lane<7;++lane) if (epoch.lanes&(1u<<lane))
        receiver_timelines_[lane].observe(double(time.time)+time.sec);
      for (auto& obs:epoch.observations) obs.lli |= LLI_SLIP;
    }
    // Do not move the accepted date anchor backwards for a late old epoch.
    if (!last_published_ || *last_published_<key) rtcm_.time=time;
    acceptCompletedEpoch(key,epoch);
  }

  void acceptCompletedEpoch(const EpochKey& key,const EpochBuffer& epoch) {
    if (epoch.observations.empty()) return;
    if (last_published_ && !(*last_published_<key)) {
      receiver_health_.observationNotMonotonic();
      return;
    }
    const auto time=gpst2time(epoch.week,epoch.tow);
    if (get_parameter("use_gps_timestamp").as_bool() && !gnss_utils::isRosTimeRepresentable(time)) {
      RCLCPP_WARN(get_logger(), "RTCM epoch outside ROS Time range: withholding measurement");
      return;
    }
    gnss_ros_standardization::msg::GnssObservations msg;
    msg.header.stamp=get_parameter("use_gps_timestamp").as_bool()
        ? gnss_utils::gpstToUtcRosTime(time) : now();
    msg.header.frame_id="gnss_receiver";
    msg.week=static_cast<uint32_t>(epoch.week); msg.tow=epoch.tow;
    msg.observations=epoch.observations;
    last_published_=key;
    obs_pub_->publish(msg);
    receiver_health_.observationCapacity(rtcm_.obs.n,MAXOBS);
    receiver_health_.published(msg.week,msg.tow,msg.observations.size());
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
        "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
        epoch.week,epoch.tow,msg.observations.size(),epoch.cnt_G,epoch.cnt_R,epoch.cnt_E,
        epoch.cnt_J,epoch.cnt_C,epoch.cnt_I,epoch.cnt_S,epoch.cnt_U);
  }

  // ---- ephemerides (snapshot-on-change + heartbeat) ------------------------
  void publishEphemeridesIfChanged(bool ingest) {
    if (rtcm_.nav.ng == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No GLONASS ephemerides (RTCM 1020). Check base settings.");
    }

    bool changed = false;
    if (ingest) for (int i = 0; i < rtcm_.nav.n; ++i) {
      changed = eph_store_.ingestEph(rtcm_.nav.eph[i]) || changed;
    }
    if (ingest) for (int i = 0; i < rtcm_.nav.ng; ++i) {
      changed = eph_store_.ingestGeph(rtcm_.nav.geph[i]) || changed;
    }

    if (changed || eph_store_.heartbeatDue(now())) {
      auto out = eph_store_.buildSnapshot(now());
    receiver_health_.snapshotRejected(eph_store_.lastSnapshotRejected());
      nav_pub_->publish(out);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Published ephemeris snapshot: GNSS=%zu GLO=%zu",
        out.gnss_ephemeris.size(), out.glonass_ephemeris.size());
    }
  }

private:
  // pubs/timer
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssEphemerides>::SharedPtr   nav_pub_;
  rclcpp::TimerBase::SharedPtr                                         timer_;

  // stream/decoder
  stream_t  stream_{};
  rtcm_t    rtcm_{};
  int station_id_{-1};
  int msm_iod_{-1};
  bool have_absolute_date_{false};
  std::map<int,std::vector<uint8_t>> unreferenced_gps_;

  // RTCM relay output (STRSVR-style): forward raw source bytes to a TCP client.
  // Capped carry absorbs partial/failed writes (see RELAY_CARRY_MAX).
  stream_t  relay_out_{};
  bool      relay_enabled_{false};
  decoder_common::CarriedStreamWriter relay_writer_{RELAY_CARRY_MAX};
  std::vector<uint8_t> rx_;
  size_t    rx_off_{0};   // read head into rx_ (see decodeRtcm3FromBuffer)

  std::chrono::steady_clock::time_point last_data_{std::chrono::steady_clock::now()};
  std::optional<EpochKey> last_published_;
  uint32_t excluded_lanes_=0;
  // epoch buffer
  std::map<EpochKey, EpochBuffer> pending_msm_epochs_;
  std::array<std::optional<gtime_t>,7> pending_msm_times_{};

  // Unified ephemeris store
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::RtcmDecoderNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("rtcm_decoder_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
