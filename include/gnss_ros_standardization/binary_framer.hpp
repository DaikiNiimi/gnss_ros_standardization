#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>
#include <string>

namespace gnss_ros_standardization::wire {

enum class Protocol { Ubx, Sbf, Novatel };
struct Frame {
  uint8_t cls{0};
  uint16_t id{0};
  uint8_t revision{0};
  uint16_t week{0};
  uint32_t tow_ms{0};
  uint8_t time_status{0};
  bool short_header{false};
  std::vector<uint8_t> payload;
  std::vector<uint8_t> bytes; // Complete integrity-checked frame for the raw decoder.
};
inline uint16_t u16(const uint8_t *p) { return p[0] | (uint16_t(p[1]) << 8); }
inline uint32_t u32(const uint8_t *p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) |
         (uint32_t(p[3]) << 24);
}
inline uint32_t crc32(const uint8_t *p, size_t n) {
  uint32_t crc = 0;
  while (n--) {
    crc ^= *p++;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0u);
  }
  return crc;
}
inline uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t crc = 0;
  while (n--) {
    crc ^= uint16_t(*p++) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
  }
  return crc;
}

// Shared by drivers and passive decoders. No payload is dispatched before its
// complete frame has passed integrity checks. Storage is bounded by the wire
// length fields. expire() discards an incomplete prefix after an input timeout;
// complete frames already buffered behind it are then recovered by drain().
class Framer {
public:
  explicit Framer(Protocol protocol) : protocol_(protocol) {}
  template <class Emit> void feed(uint8_t byte, Emit emit) {
    bytes_.push_back(byte);
    drain(emit, [](const std::string &) {});
  }
  template <class Emit, class Text> void feed(uint8_t byte, Emit emit, Text text) {
    bytes_.push_back(byte);
    drain(emit, text);
  }
  template <class Emit> void expire(Emit emit) {
    // Buffered bytes belonged to an incomplete binary candidate. Recover binary
    // syncs, but never reinterpret that candidate as text.
    quarantined_ = buffered();
    line_.clear();
    line_start_ = true;
    if (buffered() != 0) {
      ++rejected_; ++expired_;
      consume(1);
    }
    drain(emit, [](const std::string &) {}, true);
  }
  void reset() { bytes_.clear(); offset_ = 0; quarantined_ = 0; line_.clear(); line_start_ = true; }
  uint64_t rejected() const { return rejected_; }
  uint64_t checksumRejected() const { return checksum_rejected_; }
  uint64_t expired() const { return expired_; }
  uint64_t malformed() const { return malformed_; }
  size_t buffered() const { return bytes_.size() - offset_; }

private:
  void consume(size_t count) {
    offset_ += count;
    if (offset_ == bytes_.size()) { bytes_.clear(); offset_=0; }
    else if (offset_ >= 4096 && offset_ >= bytes_.size()/2) {
      bytes_.erase(bytes_.begin(),bytes_.begin()+offset_);
      offset_=0;
    }
    quarantined_ -= std::min(quarantined_, count);
  }
  template <class Text> void textByte(uint8_t byte, Text text) {
    // Only a sentence at an ASCII line boundary can be NMEA. In particular,
    // "$G..." inside a receiver command echo/reply is not another sentence.
    if (line_start_ && byte == '$') line_ = "$";
    else if (!line_.empty()) line_.push_back(static_cast<char>(byte));
    if (line_.size() > 256 || (byte < 32 && byte != '\r' && byte != '\n') || byte > 126)
      line_.clear();
    if (byte == '\n') {
      if (line_.size() >= 7 && line_[6] == ',' &&
          std::all_of(line_.begin() + 1, line_.begin() + 6,
                      [](char c) { return c >= 'A' && c <= 'Z'; })) text(line_);
      line_.clear();
    }
    line_start_ = byte == '\n';
  }
  template <class Emit, class Text> void drain(Emit emit, Text text, bool expired = false) {
    while (buffered() != 0) {
      const auto *p = bytes_.data() + offset_;
      const size_t n = buffered();
      bool bad = false;
      size_t header = 0, total = 0;
      const uint8_t first = protocol_ == Protocol::Ubx   ? 0xB5
                            : protocol_ == Protocol::Sbf ? 0x24
                                                         : 0xAA;
      if (p[0] != first) {
        if (quarantined_ == 0) textByte(p[0], text);
        consume(1);
        continue;
      }
      if (n < 2)
        break;
      if (p[1] != (protocol_ == Protocol::Ubx   ? 0x62
                   : protocol_ == Protocol::Sbf ? 0x40
                                                : 0x44))
        bad = true;
      // SBF and NMEA share '$'. A non-'@' successor is ordinary text.
      if (bad && protocol_ == Protocol::Sbf) {
        if (quarantined_ == 0) textByte(p[0], text);
        consume(1);
        continue;
      }
      if (!bad) { line_.clear(); line_start_ = true; }
      if (!bad && protocol_ == Protocol::Ubx && n >= 6) {
        header = 6;
        total = header + u16(p + 4) + 2;
      } else if (!bad && protocol_ == Protocol::Sbf && n >= 8) {
        header = 8;
        total = u16(p + 6);
        bad = total < 8 || total % 4 != 0;
      } else if (!bad && protocol_ == Protocol::Novatel && n >= 3) {
        bad = p[2] != 0x12 && p[2] != 0x13;
        if (!bad && p[2] == 0x13 && n >= 4) {
          header = 12;
          total = header + p[3] + 4;
        } else if (!bad && p[2] == 0x12 && n >= 4) {
          header = p[3];
          bad = header < 28;
          if (n >= 10)
            total = header + u16(p + 8) + 4;
        }
      }
      if (!bad && (total == 0 || n < total)) {
        if (!expired)
          break;
        bad = true;
      }
      const bool integrity_candidate = !bad;
      if (!bad) {
        if (protocol_ == Protocol::Ubx) {
          uint8_t a = 0, b = 0;
          for (size_t i = 2; i < total - 2; ++i) {
            a += p[i];
            b += a;
          }
          bad = a != p[total - 2] || b != p[total - 1];
        } else if (protocol_ == Protocol::Sbf) {
          bad = crc16(p + 4, total - 4) != u16(p + 2);
        } else {
          bad = crc32(p, total - 4) != u32(p + total - 4);
        }
      }
      if (bad) {
        ++rejected_;
        if (integrity_candidate) ++checksum_rejected_;
        else if (expired) ++expired_;
        else ++malformed_;
        quarantined_ = std::max(quarantined_, total && total <= n ? total : n);
        consume(1);
        continue;
      }
      Frame frame;
      size_t tail = 0;
      if (protocol_ == Protocol::Ubx) {
        frame.cls = p[2];
        frame.id = p[3];
        tail = 2;
      } else if (protocol_ == Protocol::Sbf) {
        frame.id = u16(p + 4) & 0x1FFF;
        frame.revision = p[5] >> 5;
      } else {
        frame.id = u16(p + 4);
        tail = 4;
        const bool short_header = p[2] == 0x13;
        frame.short_header = short_header;
        frame.week = u16(p + (short_header ? 6 : 14));
        frame.tow_ms = u32(p + (short_header ? 8 : 16));
        frame.time_status = short_header ? 0 : p[13];
        // Responses/ASCII messages are not binary measurement payloads.
        if (!short_header && (p[6] & 0xE0)) {
          consume(total);
          continue;
        }
      }
      frame.payload.assign(p + header, p + total - tail);
      frame.bytes.assign(p, p + total);
      consume(total);
      emit(frame);
    }
  }
  Protocol protocol_;
  std::vector<uint8_t> bytes_;
  uint64_t rejected_{0},checksum_rejected_{0},expired_{0},malformed_{0};
  size_t quarantined_{0}, offset_{0};
  std::string line_;
  bool line_start_{true};
};

inline bool matchesAck(const Frame &f, uint8_t cls, uint8_t id) {
  return f.cls == 0x05 && (f.id == 0 || f.id == 1) && f.payload.size() == 2 &&
         f.payload[0] == cls && f.payload[1] == id;
}
} // namespace gnss_ros_standardization::wire

// ---- receiver time quality ----
#include <chrono>
namespace gnss_ros_standardization::wire {
// N2 / Q3: match the pinned RTKLIB OEM decoder. UNKNOWN (20) and
// week zero are unusable; COARSE and FREEWHEELING retain receiver time.
// Short headers carry a time but no quality field; do not invent one from
// wall-clock age or from a previously received full header.
class NovatelTimeQuality {
public:
  using Clock = std::chrono::steady_clock;
  bool observe(const Frame &frame, Clock::time_point = Clock::now()) const {
    return frame.week > 0 && frame.tow_ms < 604800000 &&
           (frame.short_header || frame.time_status != 20);
  }
  void reset() {}
};
} // namespace gnss_ros_standardization::wire
