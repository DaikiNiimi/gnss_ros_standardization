#include "encode.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "multipath.hpp"

namespace rtk_bag_plot {
namespace {

constexpr std::string_view kDataMarker = "<!--RTK_BAG_PLOT_DATA-->";
constexpr size_t kMaxColumnPartBytes = 16U * 1024U * 1024U;

template <typename T> constexpr const char* columnType();
template <> constexpr const char* columnType<double>() { return "f64"; }
template <> constexpr const char* columnType<float>() { return "f32"; }
template <> constexpr const char* columnType<uint32_t>() { return "u32"; }
template <> constexpr const char* columnType<uint16_t>() { return "u16"; }
template <> constexpr const char* columnType<int16_t>() { return "i16"; }
template <> constexpr const char* columnType<uint8_t>() { return "u8"; }

struct Column {
  std::string name;
  std::string type;
  size_t count{0};
  std::vector<uint8_t> bytes;
};

class ColumnStore {
 public:
  template <typename T>
  std::string add(const std::string& name, const std::vector<T>& values) {
    Column column;
    column.name = name;
    column.type = columnType<T>();
    column.count = values.size();
    column.bytes.resize(values.size() * sizeof(T));
    if (!values.empty()) std::memcpy(column.bytes.data(), values.data(), column.bytes.size());
    columns_.push_back(std::move(column));
    return jsonString(name);
  }

  std::string manifest() const {
    std::string out = "[";
    for (size_t i = 0; i < columns_.size(); ++i) {
      if (i) out += ",";
      out += "{\"name\":" + jsonString(columns_[i].name) + ",\"type\":" + jsonString(columns_[i].type) +
             ",\"count\":" + std::to_string(columns_[i].count) + "}";
    }
    return out + "]";
  }

  void writeBlocks(FILE* fp) const {
    for (const auto& column : columns_) {
      size_t offset = 0;
      int part = 0;
      do {
        const size_t size = std::min(column.bytes.size() - offset, kMaxColumnPartBytes);
        std::fprintf(fp, "<script type=\"text/plain\" data-column=\"%s\" data-part=\"%d\">",
                     column.name.c_str(), part++);
        const std::string text = base64Encode(column.bytes.data() + offset, size);
        std::fwrite(text.data(), 1, text.size(), fp);
        std::fputs("</script>\n", fp);
        offset += size;
      } while (offset < column.bytes.size());
    }
  }

 private:
  std::vector<Column> columns_;
};

// Minimal JSON object builder with insertion order.
class Object {
 public:
  Object& raw(const std::string& key, const std::string& value) {
    if (!text_.empty()) text_ += ",";
    text_ += jsonString(key) + ":" + value;
    return *this;
  }
  Object& str(const std::string& key, const std::string& value) { return raw(key, jsonString(value)); }
  Object& num(const std::string& key, double value) { return raw(key, jsonNumber(value)); }
  Object& flag(const std::string& key, bool value) { return raw(key, value ? "true" : "false"); }
  std::string build() const { return "{" + text_ + "}"; }

 private:
  std::string text_;
};

template <typename Container, typename Function>
std::string jsonArray(const Container& items, Function element) {
  std::string out = "[";
  bool first = true;
  for (const auto& item : items) {
    if (!first) out += ",";
    first = false;
    out += element(item);
  }
  return out + "]";
}

std::string statsJson(const StatsTable& table) {
  return jsonArray(table, [](const std::array<AxisStats, 3>& row) {
    return jsonArray(row, [](const AxisStats& s) {
      return "[" + std::to_string(s.n) + "," + jsonNumber(s.ave) + "," + jsonNumber(s.std) + "," +
             jsonNumber(s.rms) + "]";
    });
  });
}

std::string timePosJson(ColumnStore& store, const std::string& prefix, const TimePos& series) {
  std::vector<float> x(series.x.begin(), series.x.end()), y(series.y.begin(), series.y.end()),
      z(series.z.begin(), series.z.end());
  std::vector<uint8_t> q(series.q.begin(), series.q.end());
  return Object()
      .raw("t", store.add(prefix + ".t", series.t))
      .raw("x", store.add(prefix + ".x", x))
      .raw("y", store.add(prefix + ".y", y))
      .raw("z", store.add(prefix + ".z", z))
      .raw("q", store.add(prefix + ".q", q))
      .build();
}

std::string dopJson(ColumnStore& store, const std::string& prefix, const DopSeries& dop) {
  const std::string p = prefix + ".m" + std::to_string(dop.mask);
  return Object()
      .num("mask", dop.mask)
      .raw("g", store.add(p + ".g", dop.gdop))
      .raw("p", store.add(p + ".p", dop.pdop))
      .raw("h", store.add(p + ".h", dop.hdop))
      .raw("v", store.add(p + ".v", dop.vdop))
      .raw("ns", store.add(p + ".ns", dop.ns))
      .build();
}

std::string topicJson(const TopicSummary& t) {
  return Object()
      .str("name", t.name)
      .str("type", t.type)
      .num("count", static_cast<double>(t.count))
      .num("first", t.first)
      .num("last", t.last)
      .num("hz", t.rate_hz)
      .num("max_gap", t.max_gap)
      .num("gap_count", static_cast<double>(t.gap_count))
      .num("out_of_order", static_cast<double>(t.out_of_order))
      .num("duplicates", static_cast<double>(t.duplicates))
      .num("invalid", static_cast<double>(t.invalid))
      .flag("gnss", t.gnss_epochs)
      .raw("gaps", jsonArray(t.gaps, [](const Gap& g) {
        return "[" + jsonNumber(g.start) + "," + jsonNumber(g.duration) + "]";
      }))
      .build();
}

std::string solutionJson(ColumnStore& store, size_t index, const SolutionResult& s) {
  const std::string p = "s" + std::to_string(index);
  return Object()
      .str("topic", s.topic)
      .str("source", s.source)
      .num("total", static_cast<double>(s.total_rows))
      .raw("t", store.add(p + ".t", s.t))
      .raw("e", store.add(p + ".e", s.e))
      .raw("n", store.add(p + ".n", s.n))
      .raw("u", store.add(p + ".u", s.u))
      .raw("q", store.add(p + ".q", s.q))
      .raw("ns", store.add(p + ".ns", s.ns))
      .raw("ratio", store.add(p + ".ratio", s.ratio))
      .raw("age", store.add(p + ".age", s.age))
      .raw("gdop", store.add(p + ".gdop", s.gdop))
      .raw("pdop", store.add(p + ".pdop", s.pdop))
      .raw("hdop", store.add(p + ".hdop", s.hdop))
      .raw("vdop", store.add(p + ".vdop", s.vdop))
      .raw("sd_e", store.add(p + ".sd_e", s.sd_e))
      .raw("sd_n", store.add(p + ".sd_n", s.sd_n))
      .raw("sd_u", store.add(p + ".sd_u", s.sd_u))
      .raw("vel_e", store.add(p + ".vel_e", s.vel_e))
      .raw("vel_n", store.add(p + ".vel_n", s.vel_n))
      .raw("vel_u", store.add(p + ".vel_u", s.vel_u))
      .raw("velocity", timePosJson(store, p + ".vel", s.velocity))
      .raw("acceleration", timePosJson(store, p + ".acc", s.acceleration))
      .raw("pos_stats", statsJson(s.pos_stats))
      .raw("vel_stats", statsJson(s.vel_stats))
      .raw("acc_stats", statsJson(s.acc_stats))
      .raw("status_counts", jsonArray(s.status_counts, [](size_t v) { return std::to_string(v); }))
      .build();
}

std::string observationJson(ColumnStore& store, size_t index, const ObservationResult& o) {
  const std::string p = "o" + std::to_string(index);

  std::vector<uint32_t> sat_epoch, sat_offset;
  std::vector<uint16_t> az, el;
  std::vector<float> iono;
  for (const auto& series : o.sats) {
    sat_offset.push_back(static_cast<uint32_t>(sat_epoch.size()));
    sat_epoch.insert(sat_epoch.end(), series.epoch.begin(), series.epoch.end());
    az.insert(az.end(), series.az.begin(), series.az.end());
    el.insert(el.end(), series.el.begin(), series.el.end());
    iono.insert(iono.end(), series.iono.begin(), series.iono.end());
  }
  std::vector<uint32_t> signal_epoch, signal_offset;
  std::vector<uint8_t> snr, flags;
  std::vector<int16_t> mp;
  for (const auto& signal : o.signals) {
    signal_offset.push_back(static_cast<uint32_t>(signal_epoch.size()));
    signal_epoch.insert(signal_epoch.end(), signal.epoch.begin(), signal.epoch.end());
    snr.insert(snr.end(), signal.snr.begin(), signal.snr.end());
    mp.insert(mp.end(), signal.mp.begin(), signal.mp.end());
    flags.insert(flags.end(), signal.flags.begin(), signal.flags.end());
  }

  size_t sat_cursor = 0;
  const std::string sats = jsonArray(o.sats, [&](const SatelliteSeries& s) {
    const size_t offset = sat_offset[sat_cursor++];
    return Object().str("id", s.id).num("sys", s.sys).num("off", static_cast<double>(offset))
        .num("n", static_cast<double>(s.epoch.size())).build();
  });
  size_t signal_cursor = 0;
  const std::string signals = jsonArray(o.signals, [&](const SignalSeries& s) {
    const size_t offset = signal_offset[signal_cursor++];
    return Object().num("sat", s.sat_index).str("code", s.code).num("band", s.band)
        .num("off", static_cast<double>(offset)).num("n", static_cast<double>(s.epoch.size()))
        .build();
  });
  std::vector<std::string> dops;
  for (const auto& dop : o.dops)
    if (!dop.ns.empty()) dops.push_back(dopJson(store, p, dop));

  return Object()
      .str("topic", o.topic)
      .flag("geometry", o.has_geometry)
      .num("rows", static_cast<double>(o.total_rows))
      .num("kept", static_cast<double>(o.kept_rows))
      .raw("t", store.add(p + ".t", o.t))
      .raw("sats", sats)
      .raw("signals", signals)
      .raw("sat_epoch", store.add(p + ".sat_epoch", sat_epoch))
      .raw("az", store.add(p + ".az", az))
      .raw("el", store.add(p + ".el", el))
      .raw("iono", store.add(p + ".iono", iono))
      .raw("sig_epoch", store.add(p + ".sig_epoch", signal_epoch))
      .raw("snr", store.add(p + ".snr", snr))
      .raw("mp", store.add(p + ".mp", mp))
      .raw("flags", store.add(p + ".flags", flags))
      .raw("dops", jsonArray(dops, [](const std::string& d) { return d; }))
      .build();
}

std::string paletteJson(const std::vector<Rgb>& colors) {
  return jsonArray(colors, [](const Rgb& c) {
    char text[16];
    std::snprintf(text, sizeof(text), "\"#%02x%02x%02x\"", c.r, c.g, c.b);
    return std::string(text);
  });
}

std::string palettesJson() {
  std::vector<Rgb> snr, mp, iono;
  for (int i = 0; i < 256; ++i) snr.push_back(snrColor(i * kSnrScale));
  for (int i = 0; i <= 160; ++i) mp.push_back(mpColor(-0.8 + i * 0.01));
  for (int i = 0; i <= 140; ++i) iono.push_back(ionoColor(i * 0.5));
  std::vector<Rgb> marker(markerColors().begin(), markerColors().end());
  std::vector<Rgb> marker2(secondMarkerColors().begin(), secondMarkerColors().end());
  return Object()
      .raw("snr", paletteJson(snr))
      .raw("mp", paletteJson(mp))
      .raw("iono", paletteJson(iono))
      .raw("marker", paletteJson(marker))
      .raw("marker2", paletteJson(marker2))
      .build();
}

std::string manifestJson(const PlotModel& model, ColumnStore& store) {
  Object root;
  root.str("bag", model.bag)
      .str("time_basis", model.time_basis)
      .num("origin_gpst", model.origin_gpst)
      .num("leap", model.leap_seconds)
      .str("solution_origin_topic", model.solution_origin_topic)
      .raw("solution_origin_llh", jsonArray(model.solution_origin_llh,
                                            [](double v) { return jsonNumber(v); }))
      .raw("palettes", palettesJson())
      .raw("masks", jsonArray(kElevationMasks, [](int m) { return std::to_string(m); }))
      .raw("notes", jsonArray(model.notes, [](const std::string& n) { return jsonString(n); }))
      .raw("topics", jsonArray(model.topics, topicJson));

  size_t index = 0;
  root.raw("solutions", jsonArray(model.solutions, [&](const SolutionResult& s) {
    return solutionJson(store, index++, s);
  }));
  index = 0;
  root.raw("differences", jsonArray(model.differences, [&](const SolutionDifference& d) {
    const std::string prefix = "d" + std::to_string(index++);
    return Object().num("a", static_cast<double>(d.a)).num("b", static_cast<double>(d.b))
        .raw("series", timePosJson(store, prefix, d.diff))
        .raw("stats", statsJson(d.stats))
        .build();
  }));
  index = 0;
  root.raw("observations", jsonArray(model.observations, [&](const ObservationResult& o) {
    return observationJson(store, index++, o);
  }));

  std::vector<uint16_t> eph_sat;
  std::vector<double> eph_transmit, eph_reference;
  std::vector<uint8_t> eph_unhealthy;
  for (const auto& e : model.ephemerides) {
    eph_sat.push_back(static_cast<uint16_t>(e.sat));
    eph_transmit.push_back(e.transmit);
    eph_reference.push_back(e.reference);
    eph_unhealthy.push_back(e.unhealthy ? 1 : 0);
  }
  root.raw("ephemeris", Object()
      .raw("sats", jsonArray(model.ephemeris_sats, [](const std::string& s) { return jsonString(s); }))
      .raw("sat", store.add("eph.sat", eph_sat))
      .raw("transmit", store.add("eph.transmit", eph_transmit))
      .raw("reference", store.add("eph.reference", eph_reference))
      .raw("unhealthy", store.add("eph.unhealthy", eph_unhealthy))
      .build());

  index = 0;
  root.raw("imus", jsonArray(model.imus, [&](const ImuResult& imu) {
    const std::string p = "i" + std::to_string(index++);
    Object object;
    object.str("topic", imu.topic).num("total", static_cast<double>(imu.total_rows))
        .raw("t", store.add(p + ".t", imu.t));
    static constexpr std::array<const char*, 6> axes{"ax", "ay", "az", "gx", "gy", "gz"};
    for (size_t a = 0; a < axes.size(); ++a) {
      object.raw(std::string(axes[a]) + "_min", store.add(p + "." + axes[a] + "_min", imu.min[a]));
      object.raw(std::string(axes[a]) + "_max", store.add(p + "." + axes[a] + "_max", imu.max[a]));
    }
    return object.build();
  }));

  if (model.has_residuals) {
    const auto& r = model.residuals;
    index = 0;
    std::vector<std::string> dops;
    for (const auto& dop : r.dops) dops.push_back(dopJson(store, "r", dop));
    root.raw("residuals", Object()
        .str("path", r.path)
        .num("total", static_cast<double>(r.total_rows))
        .raw("sats", jsonArray(r.sat_ids, [](const std::string& s) { return jsonString(s); }))
        .raw("t", store.add("r.t", r.t))
        .raw("sat", store.add("r.sat", r.sat_index))
        .raw("frq", store.add("r.frq", r.frequency))
        .raw("flags", store.add("r.flags", r.flags))
        .raw("az", store.add("r.az", r.az))
        .raw("el", store.add("r.el", r.el))
        .raw("resp", store.add("r.resp", r.code_residual))
        .raw("resc", store.add("r.resc", r.phase_residual))
        .raw("snr", store.add("r.snr", r.snr))
        .raw("dop_t", store.add("r.dop_t", r.dop_t))
        .raw("dops", jsonArray(dops, [](const std::string& d) { return d; }))
        .build());
  } else {
    root.raw("residuals", "null");
  }

  root.raw("columns", store.manifest());
  return root.build();
}

}  // namespace

std::string base64Encode(const uint8_t* data, size_t size) {
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((size + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < size; i += 3) {
    const uint32_t v = (uint32_t(data[i]) << 16) | (uint32_t(data[i + 1]) << 8) | data[i + 2];
    out += table[(v >> 18) & 63];
    out += table[(v >> 12) & 63];
    out += table[(v >> 6) & 63];
    out += table[v & 63];
  }
  if (i + 1 == size) {
    const uint32_t v = uint32_t(data[i]) << 16;
    out += table[(v >> 18) & 63];
    out += table[(v >> 12) & 63];
    out += "==";
  } else if (i + 2 == size) {
    const uint32_t v = (uint32_t(data[i]) << 16) | (uint32_t(data[i + 1]) << 8);
    out += table[(v >> 18) & 63];
    out += table[(v >> 12) & 63];
    out += table[(v >> 6) & 63];
    out += '=';
  }
  return out;
}

std::string jsonString(const std::string& value) {
  std::string out = "\"";
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      case '<': out += "\\u003c"; break;
      case '>': out += "\\u003e"; break;
      case '&': out += "\\u0026"; break;
      default:
        if (c < 0x20) {
          char text[8];
          std::snprintf(text, sizeof(text), "\\u%04x", c);
          out += text;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out + "\"";
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) return "null";
  char text[40];
  std::snprintf(text, sizeof(text), "%.15g", value);
  return text;
}

void writeHtml(const PlotModel& model, const uint8_t* viewer, size_t viewer_size, FILE* out) {
  const std::string_view page(reinterpret_cast<const char*>(viewer), viewer_size);
  const auto marker = page.find(kDataMarker);
  if (marker == std::string_view::npos || page.find(kDataMarker, marker + 1) != std::string_view::npos)
    throw std::logic_error("Viewer template must contain exactly one data marker");

  ColumnStore store;
  const std::string manifest = manifestJson(model, store);

  std::fwrite(page.data(), 1, marker, out);
  std::fputs("<script type=\"application/json\" id=\"manifest\">", out);
  std::fwrite(manifest.data(), 1, manifest.size(), out);
  std::fputs("</script>\n", out);
  store.writeBlocks(out);
  const auto tail = page.substr(marker + kDataMarker.size());
  std::fwrite(tail.data(), 1, tail.size(), out);
}

}  // namespace rtk_bag_plot
