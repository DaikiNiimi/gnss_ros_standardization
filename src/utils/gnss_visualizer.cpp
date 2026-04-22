/*
 * gnss_visualizer.cpp
 * A ROS 2 node to visualize GNSS solution and satellite status using OpenCV.
 * Mimics RTKLIB's rtkplot functionality (Skyplot, SNR, Solution View).
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <set>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"

// MALIB includes
#include "rtklib.h"

using namespace std::chrono_literals;

class GnssVisualizer : public rclcpp::Node {
public:
  GnssVisualizer() : Node("gnss_visualizer") {
    this->declare_parameter("image_width", 1280);
    this->declare_parameter("image_height", 720);
    this->declare_parameter("fixed_latitude", 0.0);
    this->declare_parameter("fixed_longitude", 0.0);
    this->declare_parameter("fixed_altitude", 0.0);
    this->declare_parameter("zoom_level", 0);
    this->declare_parameter("use_gui", true);

    width_ = this->get_parameter("image_width").as_int();
    height_ = this->get_parameter("image_height").as_int();

    fixed_pos_lla_[0] = this->get_parameter("fixed_latitude").as_double();
    fixed_pos_lla_[1] = this->get_parameter("fixed_longitude").as_double();
    fixed_pos_lla_[2] = this->get_parameter("fixed_altitude").as_double();
    zoom_level_ = this->get_parameter("zoom_level").as_int();
    use_gui_ = this->get_parameter("use_gui").as_bool();

    obs_sub_ = this->create_subscription<gnss_ros_standardization::msg::GnssObservations>(
      "/gnss/observation", 10, std::bind(&GnssVisualizer::obsCallback, this, std::placeholders::_1));
    eph_sub_ = this->create_subscription<gnss_ros_standardization::msg::GnssEphemerides>(
      "/gnss/ephemeris", rclcpp::QoS(10).transient_local(), std::bind(&GnssVisualizer::ephCallback, this, std::placeholders::_1));
    sol_sub_ = this->create_subscription<gnss_ros_standardization::msg::GnssSolution>(
      "/gnss/solution", 10, std::bind(&GnssVisualizer::solCallback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("gnss_visualization/dashboard", 1);

    // Parameter callback for dynamic zoom
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
          if (p.get_name() == "zoom_level") {
            zoom_level_ = p.as_int();
            // Trigger update if desired, or wait for next obs
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    render_timer_ = this->create_wall_timer(
      10ms, std::bind(&GnssVisualizer::renderTimerCallback, this));

    if (use_gui_) {
      cv::namedWindow("GNSS Visualizer", cv::WINDOW_NORMAL);
      cv::resizeWindow("GNSS Visualizer", width_, height_);
      cv::setMouseCallback("GNSS Visualizer", GnssVisualizer::onMouse, this);
    }

    RCLCPP_INFO(this->get_logger(), "GnssVisualizer started (%dx%d). GUI: %s", width_, height_, use_gui_ ? "ON" : "OFF");
  }

  ~GnssVisualizer() {
    if (use_gui_) cv::destroyAllWindows();
  }

private:
  std::mutex data_mutex_;
  gnss_ros_standardization::msg::GnssObservations::SharedPtr last_obs_;
  std::map<int, eph_t> eph_map_;
  std::map<int, geph_t> geph_map_;
  gnss_ros_standardization::msg::GnssSolution::SharedPtr last_sol_;

  // Position history with status for color coding
  struct PosEntry { double x, y; uint8_t status; };
  std::vector<PosEntry> pos_history_;

  int width_, height_;
  double fixed_pos_lla_[3];
  int zoom_level_ = 0;
  bool use_gui_ = true;

  // ── Style Constants ──
  static constexpr int FONT_TITLE = cv::FONT_HERSHEY_DUPLEX;
  static constexpr int FONT_LABEL = cv::FONT_HERSHEY_SIMPLEX;

  const cv::Scalar COL_BG       = cv::Scalar(245, 245, 245);
  const cv::Scalar COL_PANEL_BG = cv::Scalar(55, 55, 55);
  const cv::Scalar COL_TEXT     = cv::Scalar(30, 30, 30);
  const cv::Scalar COL_TEXT_DIM = cv::Scalar(120, 120, 120);
  const cv::Scalar COL_GRID     = cv::Scalar(200, 200, 200);
  const cv::Scalar COL_DIVIDER  = cv::Scalar(180, 180, 180);

  // System colors (BGR)
  cv::Scalar sysColor(int sys) const {
    switch(sys) {
      case SYS_GPS: return cv::Scalar(50, 160, 50);     // Green
      case SYS_GLO: return cv::Scalar(0, 180, 220);     // Darker Yellow (Gold-ish)
      case SYS_GAL: return cv::Scalar(200, 50, 200);    // Purple-ish Pink
      case SYS_CMP: return cv::Scalar(0, 0, 220);       // Red (BeiDou)
      case SYS_QZS: return cv::Scalar(200, 0, 0);       // Deep Blue (QZSS)
      case SYS_IRN: return cv::Scalar(0, 100, 255);     // Red-Orange (NavIC)
      case SYS_SBS: return cv::Scalar(255, 200, 0);     // Cyan-ish (SBAS)
      default:      return cv::Scalar(130, 130, 130);
    }
  }

  std::string sysName(int sys) const {
    switch(sys) {
      case SYS_GPS: return "GPS";
      case SYS_GLO: return "GLO";
      case SYS_GAL: return "GAL";
      case SYS_CMP: return "BDS";
      case SYS_QZS: return "QZS";
      case SYS_IRN: return "IRN";
      case SYS_SBS: return "SBS";
      default:      return "?";
    }
  }

  // Solution status color (RTKLIB style, BGR)
  cv::Scalar statusColor(uint8_t status) const {
    switch(status) {
      case gnss_ros_standardization::msg::GnssSolution::STATUS_FIX:    return cv::Scalar(50, 180, 50);    // Green
      case gnss_ros_standardization::msg::GnssSolution::STATUS_FLOAT:  return cv::Scalar(0, 220, 220);    // Yellow
      case gnss_ros_standardization::msg::GnssSolution::STATUS_SINGLE: return cv::Scalar(0, 0, 255);      // Red
      case gnss_ros_standardization::msg::GnssSolution::STATUS_DGPS:   return cv::Scalar(200, 50, 50);    // Blue
      case gnss_ros_standardization::msg::GnssSolution::STATUS_SBAS:   return cv::Scalar(200, 50, 50);    // Blue
      case gnss_ros_standardization::msg::GnssSolution::STATUS_PPP:    return cv::Scalar(150, 100, 50);   // Teal
      default:                                                          return cv::Scalar(130, 130, 130);
    }
  }

  // All supported systems list (now local in functions)
  // static constexpr int ALL_SYSTEMS[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_CMP, SYS_QZS, SYS_IRN};

  rclcpp::Subscription<gnss_ros_standardization::msg::GnssObservations>::SharedPtr obs_sub_;
  rclcpp::Subscription<gnss_ros_standardization::msg::GnssEphemerides>::SharedPtr eph_sub_;
  rclcpp::Subscription<gnss_ros_standardization::msg::GnssSolution>::SharedPtr sol_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr render_timer_;

  static void onMouse(int event, int x, int y, int flags, void* userdata) {
    GnssVisualizer* viz = (GnssVisualizer*)userdata;
    if (event == cv::EVENT_LBUTTONDOWN) {
      viz->handleMouseClick(x, y);
    } else if (event == cv::EVENT_MOUSEWHEEL) {
      viz->handleMouseWheel(cv::getMouseWheelDelta(flags));
    }
  }

  void handleMouseWheel(int delta) {
    if (delta > 0) {
      zoom_level_++;
    } else if (delta < 0) {
      zoom_level_--;
    }
    // Clamp
    if (zoom_level_ > 10) zoom_level_ = 10;
    if (zoom_level_ < -5) zoom_level_ = -5;
    
    RCLCPP_INFO(this->get_logger(), "Zoom Level (Wheel): %d", zoom_level_);
    this->set_parameter(rclcpp::Parameter("zoom_level", zoom_level_));
    renderLoop(false);
  }

  void handleMouseClick(int x, int y) {
    // Position plot check
    int mid_x = width_ / 2;
    int mid_y = height_ / 2;
    cv::Rect pos_roi(0, mid_y, mid_x, mid_y);

    if (pos_roi.contains(cv::Point(x, y))) {      
      cv::Rect btn_zoom_out(pos_roi.x + pos_roi.width - 65, pos_roi.y + 4, 20, 16);
      cv::Rect btn_zoom_in (pos_roi.x + pos_roi.width - 35, pos_roi.y + 4, 20, 16);

      if (btn_zoom_out.contains(cv::Point(x, y))) {
        zoom_level_--;
      } else if (btn_zoom_in.contains(cv::Point(x, y))) {
        zoom_level_++;
      }      
      if (zoom_level_ > 10) zoom_level_ = 10;
      if (zoom_level_ < -5) zoom_level_ = -5;

      RCLCPP_INFO(this->get_logger(), "Zoom Level (Click): %d", zoom_level_);
      this->set_parameter(rclcpp::Parameter("zoom_level", zoom_level_));
      renderLoop(false);
    }
  }

  void renderTimerCallback() {
    renderLoop(true);
  }

  void obsCallback(const gnss_ros_standardization::msg::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_obs_ = msg;
  }

  void ephCallback(const gnss_ros_standardization::msg::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& e : msg->gnss_ephemeris) { eph_t eph = gnss_utils::msgToEph(e); eph_map_[eph.sat] = eph; }
    for (const auto& g : msg->glonass_ephemeris) { geph_t geph = gnss_utils::msgToGeph(g); geph_map_[geph.sat] = geph; }
  }

  void solCallback(const gnss_ros_standardization::msg::GnssSolution::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_sol_ = msg;
    if (msg->status != gnss_ros_standardization::msg::GnssSolution::STATUS_NONE) {
      pos_history_.push_back({msg->pos_enu.x, msg->pos_enu.y, msg->status});
      if (pos_history_.size() > 5000) pos_history_.erase(pos_history_.begin());
    }
  }

  struct SatAzEl { double az, el; int sys; std::string satid; int sat; };

  std::vector<SatAzEl> cached_sat_azel_;
  bool is_sat_azel_cached_ = false;
  int64_t last_sat_azel_time_ns_ = 0;

  std::vector<SatAzEl> computeSatAzEl() {
    auto now = this->now();
    if (is_sat_azel_cached_ && (now.nanoseconds() - last_sat_azel_time_ns_) < 10000000000LL) {
      return cached_sat_azel_;
    }

    std::vector<SatAzEl> result;
    double rr[3]={0}, pos_llh[3]={0};
    bool has_pos = false;
    if (last_sol_ && last_sol_->status != gnss_ros_standardization::msg::GnssSolution::STATUS_NONE) {
      rr[0]=last_sol_->pos_ecef.x; rr[1]=last_sol_->pos_ecef.y; rr[2]=last_sol_->pos_ecef.z;
      pos_llh[0]=last_sol_->latitude*D2R; pos_llh[1]=last_sol_->longitude*D2R; pos_llh[2]=last_sol_->altitude;
      has_pos=true;
    } else if (norm(fixed_pos_lla_,3) > 0.001) {
      pos_llh[0]=fixed_pos_lla_[0]*D2R; pos_llh[1]=fixed_pos_lla_[1]*D2R; pos_llh[2]=fixed_pos_lla_[2];
      pos2ecef(pos_llh, rr); has_pos=true;
    }
    if (!last_obs_ || !has_pos || norm(rr,3) < 1.0) {
      return is_sat_azel_cached_ ? cached_sat_azel_ : result;
    }

    gtime_t t_obs = gpst2time(last_obs_->week, last_obs_->tow);
    std::set<int> plotted;

    for (const auto& obs : last_obs_->observations) {
      if (plotted.count(obs.sat)) continue;
      double rs[6], dts[2], var;
      const eph_t* eph = nullptr; const geph_t* geph = nullptr;
      int sys = satsys(obs.sat, nullptr);
      if (sys == SYS_GLO) { if (geph_map_.count(obs.sat)) geph = &geph_map_.at(obs.sat); }
      else { if (eph_map_.count(obs.sat)) eph = &eph_map_.at(obs.sat); }
      if (!eph && !geph) continue;

      if (sys == SYS_GLO) { if (geph) geph2pos(t_obs, geph, rs, dts, &var); else continue; }
      else                { if (eph)  eph2pos(t_obs, eph, rs, dts, &var);  else continue; }

      double e_vec[3], azel[2];
      if (geodist(rs, rr, e_vec) <= 0.0) continue;
      satazel(pos_llh, e_vec, azel);
      if (azel[1] < 0) continue;

      result.push_back({azel[0], azel[1], sys, obs.satid, obs.sat});
      plotted.insert(obs.sat);
    }
    
    cached_sat_azel_ = result;
    is_sat_azel_cached_ = true;
    last_sat_azel_time_ns_ = now.nanoseconds();
    return result;
  }

  void renderLoop(bool process_gui = true) {
    cv::Mat canvas(height_, width_, CV_8UC3, COL_BG);

    int mid_x = width_ / 2;
    int mid_y = height_ / 2;

    renderStatus      (canvas, cv::Rect(0, 0, mid_x, mid_y));
    renderSkyplot     (canvas, cv::Rect(mid_x, 0, mid_x, mid_y));
    renderPositionPlot(canvas, cv::Rect(0, mid_y, mid_x, mid_y));
    renderSnrPlot     (canvas, cv::Rect(mid_x, mid_y, mid_x, mid_y));

    cv::line(canvas, cv::Point(mid_x, 0), cv::Point(mid_x, height_), COL_DIVIDER, 2);
    cv::line(canvas, cv::Point(0, mid_y), cv::Point(width_, mid_y), COL_DIVIDER, 2);

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", canvas).toImageMsg();
    msg->header.stamp = this->now();
    image_pub_->publish(*msg);

    if (use_gui_) {
      cv::imshow("GNSS Visualizer", canvas);
      if (process_gui) cv::waitKey(1);
    }
  }

  void drawPanelHeader(cv::Mat& img, cv::Rect roi, const std::string& title) {
    int hh = 24;
    cv::rectangle(img, cv::Rect(roi.x, roi.y, roi.width, hh), COL_PANEL_BG, -1);
    cv::putText(img, title, cv::Point(roi.x + 8, roi.y + 17), FONT_LABEL, 0.55, cv::Scalar(240,240,240), 1);
  }

  void renderStatus(cv::Mat& img, cv::Rect roi) {
    drawPanelHeader(img, roi, "Status");

    std::lock_guard<std::mutex> lock(data_mutex_);

    int y = roi.y + 45;
    int x = roi.x + 15;
    int dy = 24;

    auto text = [&](const std::string& s, cv::Scalar col = cv::Scalar(30,30,30), double sc = 0.50, int th = 1) {
      cv::putText(img, s, cv::Point(x, y), FONT_LABEL, sc, col, th);
      y += dy;
    };

    if (last_sol_) {
      std::stringstream ss;
      ss << "Week " << last_sol_->time_week << "  TOW " << std::fixed << std::setprecision(1) << last_sol_->time_tow;
      text(ss.str(), COL_TEXT, 0.50, 1);
    } else {
      text("Week ---  TOW ---", COL_TEXT_DIM, 0.50, 1);
    }

    std::string status_str = "NONE";
    cv::Scalar  status_col(130,130,130);
    bool use_fixed = false;

    if (last_sol_) {
      switch(last_sol_->status) {
        case gnss_ros_standardization::msg::GnssSolution::STATUS_FIX:    status_str="FIX";    status_col=statusColor(gnss_ros_standardization::msg::GnssSolution::STATUS_FIX);  break;
        case gnss_ros_standardization::msg::GnssSolution::STATUS_FLOAT:  status_str="FLOAT";  status_col=statusColor(gnss_ros_standardization::msg::GnssSolution::STATUS_FLOAT);  break;
        case gnss_ros_standardization::msg::GnssSolution::STATUS_SINGLE: status_str="SINGLE"; status_col=statusColor(gnss_ros_standardization::msg::GnssSolution::STATUS_SINGLE); break;
        case gnss_ros_standardization::msg::GnssSolution::STATUS_DGPS:   status_str="DGPS";   status_col=statusColor(gnss_ros_standardization::msg::GnssSolution::STATUS_DGPS);  break;
        case gnss_ros_standardization::msg::GnssSolution::STATUS_SBAS:   status_str="SBAS";   status_col=statusColor(gnss_ros_standardization::msg::GnssSolution::STATUS_SBAS);  break;
        case gnss_ros_standardization::msg::GnssSolution::STATUS_PPP:    status_str="PPP";    status_col=statusColor(gnss_ros_standardization::msg::GnssSolution::STATUS_PPP);  break;
      }
    }
    if (!last_sol_ || last_sol_->status == gnss_ros_standardization::msg::GnssSolution::STATUS_NONE) {
      if (norm(fixed_pos_lla_, 3) > 0.001) { status_str = "MANUAL"; status_col = cv::Scalar(0,120,120); use_fixed = true; }
    }

    int badge_w = 10 + (int)status_str.size() * 14;
    cv::rectangle(img, cv::Rect(x, y-18, badge_w, 24), status_col, -1);
    cv::rectangle(img, cv::Rect(x, y-18, badge_w, 24), status_col * 0.7, 1);
    cv::putText(img, status_str, cv::Point(x+5, y), FONT_LABEL, 0.6, cv::Scalar(255,255,255), 2);
    y += dy + 4;

    double lat=0, lon=0, alt=0;
    if (last_sol_ && !use_fixed) { lat=last_sol_->latitude; lon=last_sol_->longitude; alt=last_sol_->altitude; }
    else if (use_fixed) { lat=fixed_pos_lla_[0]; lon=fixed_pos_lla_[1]; alt=fixed_pos_lla_[2]; }

    {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(8);
      ss << "Lat  " << lat; text(ss.str()); ss.str("");
      ss << "Lon  " << lon; text(ss.str()); ss.str("");
      ss << std::setprecision(3) << "Alt  " << alt << " m"; text(ss.str());
    }

    if (last_sol_) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);
      double ve = last_sol_->vel_enu.x, vn = last_sol_->vel_enu.y, vu = last_sol_->vel_enu.z;
      double spd = std::sqrt(ve*ve + vn*vn + vu*vu);
      ss << "Vel E:" << ve << " N:" << vn << " U:" << vu << "  |" << spd << "| m/s";
      text(ss.str(), COL_TEXT_DIM, 0.42);
    }

    y += 2;
    int n_sol = last_sol_ ? last_sol_->num_sats : 0;
    std::map<int, int> sats_by_sys;
    int n_unique = 0;
    int n_signals = 0;
    if (last_obs_) {
      std::set<int> uniqs;
      for (const auto& o : last_obs_->observations) { uniqs.insert(o.sat); n_signals++; }
      n_unique = uniqs.size();
      for (int s : uniqs) sats_by_sys[satsys(s, nullptr)]++;
    }

    {
      std::stringstream ss;
      ss << "Sats  " << n_sol << " / " << n_unique << "  (" << n_signals << " signals)";
      text(ss.str(), COL_TEXT, 0.45, 1);
    }

    {
      int sx = x;
      const int systems[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_CMP, SYS_QZS, SYS_IRN, SYS_SBS};
      for (int sys : systems) {
        if (sats_by_sys[sys] == 0 && sys == SYS_IRN) continue;
        cv::circle(img, cv::Point(sx+4, y-5), 4, sysColor(sys), -1);
        std::string lbl = sysName(sys) + ":" + std::to_string(sats_by_sys[sys]);
        cv::putText(img, lbl, cv::Point(sx+11, y), FONT_LABEL, 0.38, COL_TEXT);
        sx += 12 + (int)lbl.size() * 7;
      }
      y += dy;
    }

    if (last_sol_) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1);
      ss << "GDOP " << last_sol_->gdop
         << "  PDOP " << last_sol_->pdop
         << "  HDOP " << last_sol_->hdop
         << "  VDOP " << last_sol_->vdop;
      text(ss.str(), COL_TEXT_DIM, 0.42);
    }

    if (last_sol_) {
      std::stringstream ss;
      bool has_content = false;
      if (last_sol_->ratio > 0.01f) {
        ss << "AR Ratio  " << std::fixed << std::setprecision(1) << last_sol_->ratio;
        has_content = true;
      }
      if (last_sol_->age_diff > 0.01f) {
        if (has_content) ss << "   ";
        ss << "Age  " << std::fixed << std::setprecision(1) << last_sol_->age_diff << "s";
        has_content = true;
      }
      if (has_content) {
        text(ss.str(), COL_TEXT_DIM, 0.42);
      }
    }
  }

  void renderSkyplot(cv::Mat& img, cv::Rect roi) {
    drawPanelHeader(img, roi, "Skyplot");

    int cx = roi.x + roi.width / 2;
    int cy = roi.y + 24 + (roi.height - 24) / 2;
    int r  = std::min(roi.width, roi.height - 24) / 2 - 30;

    for (int el_deg = 0; el_deg <= 90; el_deg += 30) {
      int ri = (int)(r * (1.0 - el_deg / 90.0));
      if (ri > 0) cv::circle(img, cv::Point(cx, cy), ri, COL_GRID, 1);
    }
    cv::line(img, cv::Point(cx-r, cy), cv::Point(cx+r, cy), COL_GRID, 1);
    cv::line(img, cv::Point(cx, cy-r), cv::Point(cx, cy+r), COL_GRID, 1);
    int d = (int)(r * 0.707);
    cv::line(img, cv::Point(cx-d, cy-d), cv::Point(cx+d, cy+d), cv::Scalar(225,225,225), 1);
    cv::line(img, cv::Point(cx+d, cy-d), cv::Point(cx-d, cy+d), cv::Scalar(225,225,225), 1);

    cv::putText(img, "N", cv::Point(cx-6, cy-r-6),  FONT_LABEL, 0.55, COL_TEXT, 1);
    cv::putText(img, "S", cv::Point(cx-5, cy+r+16), FONT_LABEL, 0.55, COL_TEXT, 1);
    cv::putText(img, "E", cv::Point(cx+r+6, cy+5),  FONT_LABEL, 0.55, COL_TEXT, 1);
    cv::putText(img, "W", cv::Point(cx-r-22, cy+5), FONT_LABEL, 0.55, COL_TEXT, 1);

    for (int el : {30, 60}) {
      int ri = (int)(r * (1.0 - el / 90.0));
      std::string lbl = std::to_string(el);
      cv::putText(img, lbl, cv::Point(cx+3, cy-ri+13), FONT_LABEL, 0.35, COL_TEXT_DIM);
    }
    cv::putText(img, "90", cv::Point(cx+3, cy+12), FONT_LABEL, 0.3, COL_TEXT_DIM);

    std::lock_guard<std::mutex> lock(data_mutex_);
    auto sat_azel = computeSatAzEl();
    if (sat_azel.empty()) return;

    struct SatPlot { int px, py, sys; std::string id; };
    std::vector<SatPlot> sat_plots;

    for (const auto& sa : sat_azel) {
      double dist = r * (1.0 - sa.el * 2.0 / M_PI);
      int px = cx + (int)(dist * sin(sa.az));
      int py = cy - (int)(dist * cos(sa.az));
      sat_plots.push_back({px, py, sa.sys, sa.satid});
    }

    for (const auto& sp : sat_plots) {
      cv::circle(img, cv::Point(sp.px, sp.py), 6, cv::Scalar(255,255,255), -1);
      cv::circle(img, cv::Point(sp.px, sp.py), 6, sysColor(sp.sys), 2);
      cv::circle(img, cv::Point(sp.px, sp.py), 4, sysColor(sp.sys), -1);
    }

    std::vector<cv::Rect> occupied;
    for (const auto& sp : sat_plots) {
      occupied.push_back(cv::Rect(sp.px - 8, sp.py - 8, 16, 16));
    }

    for (const auto& sp : sat_plots) {
      int baseline = 0;
      cv::Size tsize = cv::getTextSize(sp.id, FONT_LABEL, 0.38, 1, &baseline);
      int tw = tsize.width;
      int th = tsize.height;

      struct Cand { int dx, dy; };
      Cand cands[] = {
        {8, 4},
        {-tw - 8, 4},
        {-tw / 2, -10},
        {-tw / 2, th + 10},
        {8, -10},
        {-tw - 8, -10},
        {8, th + 10},
        {-tw - 8, th + 10}
      };

      Cand best_cand = cands[0];

      for (auto c : cands) {
        cv::Rect r(sp.px + c.dx - 2, sp.py + c.dy - th - 2, tw + 4, th + 4);
        bool collision = false;
        for (const auto& occ : occupied) {
          if ((r.x < occ.x + occ.width) && (r.x + r.width > occ.x) &&
              (r.y < occ.y + occ.height) && (r.y + r.height > occ.y)) {
            collision = true;
            break;
          }
        }
        if (!collision) {
          best_cand = c;
          break;
        }
      }

      int lx = sp.px + best_cand.dx;
      int ly = sp.py + best_cand.dy;
      occupied.push_back(cv::Rect(lx - 2, ly - th - 2, tw + 4, th + 4));
      
      // Draw text with outline (halo) for readability
      cv::putText(img, sp.id, cv::Point(lx, ly), FONT_LABEL, 0.38, COL_BG, 3);
      cv::putText(img, sp.id, cv::Point(lx, ly), FONT_LABEL, 0.38, COL_TEXT, 1);
    }

    int lx = roi.x + roi.width - 55;
    int ly = roi.y + roi.height - 95;
    const int systems[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_CMP, SYS_QZS, SYS_IRN, SYS_SBS};
    for (int sys : systems) {
      cv::circle(img, cv::Point(lx, ly), 5, sysColor(sys), -1);
      cv::putText(img, sysName(sys), cv::Point(lx+10, ly+4), FONT_LABEL, 0.38, COL_TEXT);
      ly += 14;
    }
  }

  void renderSnrPlot(cv::Mat& img, cv::Rect roi) {
    drawPanelHeader(img, roi, "Elevation / SNR");
    if (!last_obs_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);

    auto sat_azel = computeSatAzEl();
    std::map<int, double> sat_el_map;
    for (const auto& sa : sat_azel) sat_el_map[sa.sat] = sa.el * R2D;

    struct SigInfo { float snr; std::string code; };
    std::map<int, std::vector<SigInfo>> sig_by_sat;
    std::map<int, std::string> sat_ids;
    for (const auto& obs : last_obs_->observations) {
      sig_by_sat[obs.sat].push_back({obs.snr, obs.code_str});
      sat_ids[obs.sat] = obs.satid;
    }
    if (sig_by_sat.empty()) return;

    struct FreqSNR { float l1 = 0, l2 = 0, l5 = 0; };
    std::map<int, FreqSNR> freq_snr;
    for (const auto& [sat, sigs] : sig_by_sat) {
      FreqSNR& f = freq_snr[sat];
      for (const auto& s : sigs) {
        if (s.code.empty()) continue;
        char c = s.code[0];
        if (c == '1') f.l1 = std::max(f.l1, s.snr);
        else if (c == '2') f.l2 = std::max(f.l2, s.snr);
        else if (c == '5' || c == '6' || c == '7') f.l5 = std::max(f.l5, s.snr);
      }
    }

    std::vector<int> sat_list;
    for (const auto& [sat, _] : sig_by_sat) sat_list.push_back(sat);

    int n_sats   = sat_list.size();
    int hdr_h    = 24;
    int margin_l = 75; 
    int margin_r = 10;
    int margin_b = 60;
    int margin_t = 5;
    int total_h  = roi.height - hdr_h - margin_b - margin_t;
    int n_plots  = 4;
    int gap      = 12; // Increased gap between plots
    int sub_h    = (total_h - (n_plots - 1) * gap) / n_plots;
    int plot_w   = roi.width - margin_l - margin_r;
    int start_x  = roi.x + margin_l;

    double slot_w = (double)plot_w / n_sats;
    double bar_w  = slot_w * 0.7;
    if (bar_w > 20) bar_w = 20;
    if (bar_w < 2) bar_w = 2;

    struct SubPlotConfig { const char* title; double max_val; };
    SubPlotConfig configs[4] = {
      {"Elev", 90.0}, {"L1 SNR", 60.0}, {"L2 SNR", 60.0}, {"L5 SNR", 60.0}
    };

    for (int pi = 0; pi < n_plots; pi++) {
      int top_y    = roi.y + hdr_h + margin_t + pi * (sub_h + gap);
      int bottom_y = top_y + sub_h;

      cv::rectangle(img, cv::Rect(start_x, top_y, plot_w, sub_h), cv::Scalar(252,252,252), -1);

      cv::putText(img, configs[pi].title, cv::Point(roi.x + 4, top_y + sub_h / 2 + 4), FONT_LABEL, 0.32, COL_TEXT_DIM, 1);

      double max_val = configs[pi].max_val;
      int step = (pi == 0) ? 15 : 10; // Elev: 15 deg steps, SNR: 10 dBHz steps
      for (int v = 0; v <= (int)max_val; v += step) {
        int gy = bottom_y - (int)((v / max_val) * sub_h);
        cv::Scalar gc = (v % (step*2) == 0) ? COL_GRID : cv::Scalar(232,232,232);
        cv::line(img, cv::Point(start_x, gy), cv::Point(start_x + plot_w, gy), gc, 1);
        if (v > 0 && v < (int)max_val) {
          cv::putText(img, std::to_string(v), cv::Point(start_x - 22, gy + 4), FONT_LABEL, 0.28, COL_TEXT_DIM);
        }
      }

      if (pi > 0) {
        // ... threshold lines ...
        int y20 = bottom_y - (int)((20.0 / max_val) * sub_h);
        for (int dx = start_x; dx < start_x + plot_w; dx += 8)
          cv::line(img, cv::Point(dx, y20), cv::Point(std::min(dx+4, start_x+plot_w), y20), cv::Scalar(0,140,255), 1);
        int y35 = bottom_y - (int)((35.0 / max_val) * sub_h);
        for (int dx = start_x; dx < start_x + plot_w; dx += 8)
          cv::line(img, cv::Point(dx, y35), cv::Point(std::min(dx+4, start_x+plot_w), y35), cv::Scalar(60,180,60), 1);
      }

      for (int si = 0; si < n_sats; si++) {
        int sat = sat_list[si];
        int sys = satsys(sat, nullptr);
        
        // Use floating point center for layout consistency
        double cx_f = start_x + slot_w * si + slot_w / 2.0;
        int cx = (int)cx_f;

        double val = 0;
        if (pi == 0) {
          if (sat_el_map.count(sat)) val = sat_el_map[sat];
        } else if (pi == 1) val = freq_snr[sat].l1;
        else if (pi == 2) val = freq_snr[sat].l2;
        else val = freq_snr[sat].l5;

        if (val <= 0) continue;

        double h = (val / max_val) * sub_h;
        if (h > sub_h) h = sub_h;
        if (h < 1) h = 1;

        cv::Scalar col = sysColor(sys);
        // Fixed L5 color lightness issue by removing fading logic
        // Use standard colors for all bands

        int bx = cx - (int)(bar_w / 2);
        cv::Rect bar(bx, bottom_y - (int)h, (int)bar_w, (int)h);
        cv::rectangle(img, bar, col, -1);
        cv::rectangle(img, bar, col * 0.7, 1);
      }
    }

    int label_y_start = roi.y + hdr_h + margin_t + n_plots * (sub_h + gap) + 4;

    for (int si = 0; si < n_sats; si++) {
      int sat = sat_list[si];
      int sys = satsys(sat, nullptr);
      
      double cx_f = start_x + slot_w * si + slot_w / 2.0;
      int cx = (int)cx_f;

      std::string label = sat_ids[sat];
      
      // Calculate exact text size for centering
      int baseline = 0;
      cv::Size ref_size = cv::getTextSize(label, FONT_LABEL, 0.32, 1, &baseline);
      int txt_w = ref_size.width; 
      int txt_h = ref_size.height;
      
      // Create exact-size image for rotation
      // Mat(rows, cols) -> (height, width)
      cv::Mat txt_img(txt_h + 8, txt_w + 4, CV_8UC3, COL_BG);
      // Fill with BG to avoid artifacts
      txt_img = COL_BG;
      cv::putText(txt_img, label, cv::Point(2, txt_h + 4), FONT_LABEL, 0.32, sysColor(sys), 1);
      
      cv::Mat rot;
      cv::rotate(txt_img, rot, cv::ROTATE_90_CLOCKWISE);

      // Center horizontally on cx
      int tx = cx - rot.cols / 2;
      int ty = label_y_start;
      
      // Relaxed clipping: Allow render if part of label is inside ROI x-range
      // Also check bottom boundary
      if (tx + rot.cols > roi.x && tx < roi.x + roi.width && ty + rot.rows < roi.y + roi.height) {
        int st_x = std::max(0, tx);
        int ed_x = std::min(width_, tx + rot.cols);
        int st_y = std::max(0, ty);
        int ed_y = std::min(height_, ty + rot.rows);
        if (st_x < ed_x && st_y < ed_y) {
          int src_x = st_x - tx;
          int src_y = st_y - ty;
          int w = ed_x - st_x;
          int h = ed_y - st_y;
          cv::Rect src_rect(src_x, src_y, w, h);
          cv::Rect dst_rect(st_x, st_y, w, h);
          
          cv::Mat src_crop = rot(src_rect);
          cv::Mat dst_crop = img(dst_rect);
          
          cv::Mat mask;
          cv::inRange(src_crop, COL_BG, COL_BG, mask);
          cv::bitwise_not(mask, mask);
          src_crop.copyTo(dst_crop, mask);
        }
      }
    }
  }

  void renderPositionPlot(cv::Mat& img, cv::Rect roi) {
    drawPanelHeader(img, roi, "Position (ENU)");

    int hdr_h = 24;
    int cx = roi.x + roi.width / 2;
    int cy = roi.y + hdr_h + (roi.height - hdr_h) / 2;
    double scale = 5.0 * std::pow(2.0, zoom_level_);
    double grid_m = 10.0;

    if (zoom_level_ >= 9) grid_m = 0.01;
    else if (zoom_level_ >= 6) grid_m = 0.1;
    else if (zoom_level_ >= 3) grid_m = 1.0;
    else if (zoom_level_ >= 1) grid_m = 5.0;
    else if (zoom_level_ <= -2) grid_m = 50.0;
    else if (zoom_level_ <= -1) grid_m = 20.0;

    cv::Scalar gridCol(225,225,225);
    for (int gi = -100; gi <= 100; ++gi) {
      if (gi == 0) continue; // Axis is drawn separately
      int off = (int)(gi * grid_m * scale);
      int gy = cy + off;
      if (gy > roi.y + hdr_h && gy < roi.y + roi.height)
        cv::line(img, cv::Point(roi.x, gy), cv::Point(roi.x+roi.width, gy), gridCol, 1);
      int gx = cx + off;
      if (gx > roi.x && gx < roi.x + roi.width)
        cv::line(img, cv::Point(gx, roi.y+hdr_h), cv::Point(gx, roi.y+roi.height), gridCol, 1);
    }
    cv::line(img, cv::Point(roi.x, cy), cv::Point(roi.x+roi.width, cy), cv::Scalar(190,190,190), 1);
    cv::line(img, cv::Point(cx, roi.y+hdr_h), cv::Point(cx, roi.y+roi.height), cv::Scalar(190,190,190), 1);

    cv::putText(img, "E", cv::Point(roi.x + roi.width - 18, cy - 4), FONT_LABEL, 0.45, COL_TEXT_DIM);
    cv::putText(img, "N", cv::Point(cx + 4, roi.y + hdr_h + 14), FONT_LABEL, 0.45, COL_TEXT_DIM);

    {
      int sx = roi.x + 10;
      int sy = roi.y + roi.height - 15;
      int bar_px = (int)(grid_m * scale);
      if (bar_px > 5 && bar_px < roi.width - 20) {
        cv::line(img, cv::Point(sx, sy), cv::Point(sx + bar_px, sy), COL_TEXT, 2);
        cv::line(img, cv::Point(sx, sy-3), cv::Point(sx, sy+3), COL_TEXT, 1);
        cv::line(img, cv::Point(sx+bar_px, sy-3), cv::Point(sx+bar_px, sy+3), COL_TEXT, 1);
        std::stringstream ss;
        if (grid_m < 1.0) ss << std::fixed << std::setprecision(2) << grid_m << "m";
        else ss << (int)grid_m << "m";
        cv::putText(img, ss.str(), cv::Point(sx + bar_px/2 - 12, sy - 5), FONT_LABEL, 0.38, COL_TEXT);
      }
    }

    {
      std::string zlbl;
      if (zoom_level_ >= 0) zlbl = "x" + std::to_string(1 << zoom_level_);
      else zlbl = "x1/" + std::to_string(1 << (-zoom_level_));
      cv::putText(img, zlbl, cv::Point(roi.x + roi.width - 85, roi.y + hdr_h + 16), FONT_LABEL, 0.42, COL_TEXT_DIM, 1);
      
      // Draw Zoom Buttons
      cv::Rect btn_zoom_out(roi.x + roi.width - 65, roi.y + 4, 20, 16);
      cv::Rect btn_zoom_in (roi.x + roi.width - 35, roi.y + 4, 20, 16);
      
      cv::rectangle(img, btn_zoom_out, cv::Scalar(200,200,200), -1);
      cv::rectangle(img, btn_zoom_in,  cv::Scalar(200,200,200), -1);
      cv::rectangle(img, btn_zoom_out, cv::Scalar(100,100,100), 1);
      cv::rectangle(img, btn_zoom_in,  cv::Scalar(100,100,100), 1);
      
      cv::putText(img, "-", cv::Point(btn_zoom_out.x + 5, btn_zoom_out.y + 12), FONT_LABEL, 0.5, cv::Scalar(0,0,0), 1);
      cv::putText(img, "+", cv::Point(btn_zoom_in.x + 4, btn_zoom_in.y + 12), FONT_LABEL, 0.5, cv::Scalar(0,0,0), 1);
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (pos_history_.empty()) {
      cv::drawMarker(img, cv::Point(cx, cy), cv::Scalar(0,0,200), cv::MARKER_CROSS, 10, 1);
      return;
    }

    size_t n = pos_history_.size();
    cv::Point prev_p(-1, -1);
    uint8_t prev_status = 0;

    for (size_t i = 0; i < n; ++i) {
      long px_long = cx + (long)(pos_history_[i].x * scale);
      long py_long = cy - (long)(pos_history_[i].y * scale);
      
      const long MAX_COORD = 30000;
      if (px_long < -MAX_COORD || px_long > MAX_COORD || py_long < -MAX_COORD || py_long > MAX_COORD) {
        prev_p = cv::Point(-1,-1);
        continue;
      }
      cv::Point p((int)px_long, (int)py_long);
      
      cv::Scalar col = statusColor(pos_history_[i].status);
      
      if (prev_p.x != -1 && pos_history_[i].status == prev_status) {
         if (roi.contains(p) || roi.contains(prev_p)) {
           cv::line(img, prev_p, p, col, 1, cv::LINE_AA);
         }
      }
      if (roi.contains(p)) {
        cv::circle(img, p, (zoom_level_ >= 6) ? 2 : 1, col, -1, cv::LINE_AA);
      }
      
      prev_p = p;
      prev_status = pos_history_[i].status;
    }

    auto& last = pos_history_.back();
    cv::Point pCur(cx + (int)(last.x * scale), cy - (int)(last.y * scale));
    if (roi.contains(pCur)) {
      cv::Scalar cur_col = statusColor(last.status);
      cv::circle(img, pCur, 5, cur_col, -1);
      cv::circle(img, pCur, 5, cv::Scalar(255,255,255), 1);
    }

    if (last_sol_ && last_sol_->pos_enu_cov.size() >= 9) {
      double cov_ee = last_sol_->pos_enu_cov[0];
      double cov_nn = last_sol_->pos_enu_cov[4];
      double cov_en = last_sol_->pos_enu_cov[1];

      if (cov_ee > 0 && cov_nn > 0) {
        double a = cov_ee, b = cov_en, dd = cov_nn;
        double tr = a + dd;
        double det = a*dd - b*b;
        double disc = tr*tr/4.0 - det;
        if (disc >= 0) {
          double l1 = tr/2.0 + std::sqrt(disc);
          double l2 = tr/2.0 - std::sqrt(disc);
          if (l1 > 0 && l2 > 0) {
            double angle = 0.5 * std::atan2(2*b, a-dd) * 180.0 / M_PI;
            int ax1 = (int)(std::sqrt(l1) * scale);
            int ax2 = (int)(std::sqrt(l2) * scale);
            if (ax1 > 2 && ax2 > 2 && ax1 < roi.width && ax2 < roi.height) {
              cv::ellipse(img, pCur, cv::Size(ax1, ax2), -angle, 0, 360, cv::Scalar(200,100,100), 1);
            }
          }
        }
      }
    }

    {
      int lx = roi.x + roi.width - 75;
      int ly = roi.y + roi.height - 70;
      const std::pair<uint8_t, const char*> status_list[] = {
        {gnss_ros_standardization::msg::GnssSolution::STATUS_FIX, "FIX"},
        {gnss_ros_standardization::msg::GnssSolution::STATUS_FLOAT, "FLOAT"},
        {gnss_ros_standardization::msg::GnssSolution::STATUS_SINGLE, "SINGLE"},
        {gnss_ros_standardization::msg::GnssSolution::STATUS_DGPS, "DGPS"},
        {gnss_ros_standardization::msg::GnssSolution::STATUS_PPP, "PPP"},
      };
      for (const auto& [st, name] : status_list) {
        cv::circle(img, cv::Point(lx, ly), 4, statusColor(st), -1);
        cv::putText(img, name, cv::Point(lx+8, ly+4), FONT_LABEL, 0.32, COL_TEXT);
        ly += 12;
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssVisualizer>());
  rclcpp::shutdown();
  return 0;
}
