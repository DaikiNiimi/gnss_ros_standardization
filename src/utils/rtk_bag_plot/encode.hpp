// Serializes the plot model into one self-contained HTML document.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

#include "plot_data.hpp"

namespace rtk_bag_plot {

std::string base64Encode(const uint8_t* data, size_t size);

// JSON string literal, safe to embed inside an HTML script element.
std::string jsonString(const std::string& value);

// JSON number; non-finite values become null.
std::string jsonNumber(double value);

// Writes the viewer page with the model embedded.
void writeHtml(const PlotModel& model, const uint8_t* viewer, size_t viewer_size, FILE* out);

}  // namespace rtk_bag_plot
