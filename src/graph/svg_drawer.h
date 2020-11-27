// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SRC_GRAPH_SVG_DRAWER_H_
#define SRC_GRAPH_SVG_DRAWER_H_

#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace DAGSfM {

class SvgStyle {
public:
  SvgStyle()
      : fill_col_(""), stroke_col_("black"), tool_tip_(""), stroke_width_(1.0f),
        opacity_(-1.0) {}

  // Configure fill color
  SvgStyle &Fill(const std::string &fill_col) {
    fill_col_ = fill_col;
    return *this;
  }

  // Configure stroke color and width
  SvgStyle &Stroke(const std::string &stroke_col, float stroke_width = 1.f) {
    stroke_col_ = stroke_col;
    stroke_width_ = stroke_width;
    return *this;
  }

  // Configure with no stroke
  SvgStyle &NoStroke() {
    stroke_col_ = "";
    stroke_width_ = 0.f;
    return *this;
  }

  // Configure tooltip
  SvgStyle &ToolTip(const std::string &tool_tip) {
    tool_tip_ = tool_tip;
    return *this;
  }

  SvgStyle &Opacity(const float opacity) {
    opacity_ = opacity;
    return *this;
  }

  const std::string GetSvgStream() const {
    std::ostringstream os;

    if (!stroke_col_.empty()) {
      os << " stroke=\"" << stroke_col_ << "\" stroke-width=\"" << stroke_width_
         << "\"";
    }
    if (fill_col_.empty()) {
      os << " fill=\"none\"";
    } else {
      os << " fill=\"" << fill_col_ << "\"";
    }

    if (opacity_ > 0) {
      os << " opacity=\"" << opacity_ << "\"";
    }

    if (!tool_tip_.empty()) {
      os << " tooltip=\"enable\">"
         << "<title>" << tool_tip_ << "</title>";
    }

    return os.str();
  }

  bool bTooltip() const { return !tool_tip_.empty(); }

private:
  std::string fill_col_, stroke_col_, tool_tip_;
  float stroke_width_;
  float opacity_; // Must be in the range [0; 1]
};

/// Basic class to handle simple SVG draw.
/// You can draw line, square, rectangle, text and image (xlink)
class SvgDrawer {
public:
  /// Constructor
  SvgDrawer(size_t W = 0, size_t H = 0) {
    svg_stream_ << "<?xml version=\"1.0\" standalone=\"yes\"?>\n";

    svg_stream_ << "<!-- SVG graphic -->" << std::endl
                << "<svg xmlns='http://www.w3.org/2000/svg'"
                << " xmlns:xlink='http://www.w3.org/1999/xlink'"
                << "\n";

    if (W > 0 && H > 0)
      svg_stream_ << "width=\"" << W << "px\" height=\"" << H << "px\""
                  << " preserveAspectRatio=\"xMinYMin meet\""
                  << " viewBox=\"0 0 " << W << ' ' << H << "\"";

    svg_stream_ << " version=\"1.1\">" << std::endl;
  }

  /// Circle draw -> x,y position and radius
  void DrawCircle(float cx, float cy, float r, const SvgStyle &style) {
    svg_stream_ << "<circle cx=\"" << cx << "\""
                << " cy=\"" << cy << "\""
                << " r=\"" << r << "\""
                << style.GetSvgStream() +
                       (style.bTooltip() ? "</circle>" : "/>\n");
  }

  /// Line draw -> start and end point
  void DrawLine(float ax, float ay, float bx, float by, const SvgStyle &style) {
    svg_stream_ << "<polyline points=\"" << ax << "," << ay << "," << bx << ","
                << by << "\""
                << style.GetSvgStream() +
                       (style.bTooltip() ? "</polyline>" : "/>\n");
  }

  /// Reference to an image (path must be relative to the svg file)
  void DrawImage(const std::string &image_path, int W, int H, int pos_x = 0,
                 int pos_y = 0, float opacity = 1.f) {
    svg_stream_ << "<image x=\"" << pos_x << "\""
                << " y=\"" << pos_y << "\""
                << " width=\"" << W << "px\""
                << " height=\"" << H << "px\""
                << " opacity=\"" << opacity << "\""
                << " xlink:href=\"" << image_path << "\""
                << "/>\n";
  }

  /// Square draw -> x,y position and size
  void DrawSquare(float cx, float cy, float W, const SvgStyle &style) {
    DrawRectangle(cx, cy, W, W, style);
  }

  /// Circle draw -> x,y position and width and height
  void DrawRectangle(float cx, float cy, float W, float H,
                     const SvgStyle &style) {
    svg_stream_ << "<rect x=\"" << cx << "\""
                << " y=\"" << cy << "\""
                << " width=\"" << W << "\""
                << " height=\"" << H << "\""
                << style.GetSvgStream() +
                       (style.bTooltip() ? "</rect>" : "/>\n");
  }

  /// Text display -> x,y position, font size
  void DrawText(float cx, float cy, float font_size = 1.0f,
                const std::string &text = "", const std::string &col = "") {
    svg_stream_ << "<text"
                << " x=\"" << cx << "\""
                << " y=\"" << cy << "\""
                << " font-size=\"" << font_size << "\"";
    if (!col.empty()) {
      svg_stream_ << " fill=\"" << col << "\"";
    }

    svg_stream_ << ">" << text << "</text>\n";
  }

  template <typename DataInputIteratorX, typename DataInputIteratorY>
  void DrawPolyline(DataInputIteratorX x_start, DataInputIteratorX x_end,
                    DataInputIteratorY y_start, DataInputIteratorY y_end,
                    const SvgStyle &style) {
    svg_stream_ << "<polyline points=\"";

    DataInputIteratorY itery = y_start;
    for (DataInputIteratorX iterx = x_start; iterx != x_end;
         std::advance(iterx, 1), std::advance(itery, 1)) {
      svg_stream_ << *iterx << ',' << *itery << ' ';
    }
    svg_stream_ << "\""
                << style.GetSvgStream() +
                       (style.bTooltip() ? "</polyline>" : "/>\n");
  }

  template <typename DataInputIteratorXY>
  void DrawPolyline(DataInputIteratorXY points, const SvgStyle &style) {
    svg_stream_ << "<polyline points=\"";

    std::copy(points.cbegin(), points.cend(),
              std::ostream_iterator<typename DataInputIteratorXY::value_type>(
                  svg_stream_, ","));

    svg_stream_ << "\""
                << style.GetSvgStream() +
                       (style.bTooltip() ? "</polyline>" : "/>\n");
  }

  /// Close the svg tag.
  std::ostringstream &CloseSvgFile() {
    svg_stream_ << "</svg>";
    return svg_stream_;
  }

private:
  std::ostringstream svg_stream_;
};

/// Helper to draw a SVG histogram
/// ____
/// |  |   ___ |
/// |  |__|  | |
/// |  |  |  | |
/// -----------|
struct SvgHistogram {
  template <typename T> static std::string stringifier(const T &t) {
    std::ostringstream os;
    os << t;
    return os.str();
  }

  template <typename T>
  void Draw(const std::vector<T> &vec_value,
            const std::pair<float, float> &range, const std::string &filename,
            const float W, const float H) {
    if (vec_value.empty()) {
      return;
    }
    //-- Max value
    const T maxi = *max_element(vec_value.begin(), vec_value.end());
    const size_t n = vec_value.size();

    const float scale_factor_y = H / static_cast<float>(maxi);
    const float scale_factor_x = W / static_cast<float>(n);

    SvgDrawer svg_stream;

    for (size_t i = 0; i < vec_value.size(); ++i) {
      const size_t dist = i;
      const T val = vec_value[i];
      std::ostringstream os;
      os << '(' << range.first + dist / float(n) * (range.second - range.first)
         << ',' << val << ')';
      SvgStyle style =
          SvgStyle().Fill("blue").Stroke("black", 1.0).ToolTip(os.str());
      svg_stream.DrawRectangle(scale_factor_x * dist, H - val * scale_factor_y,
                               scale_factor_x, val * scale_factor_y, style);
      //_________
      //|       |_________
      //|       ||       |
      //|       ||       |
      //|       ||       |
      // 0    sFactorX  2*sFactorX
    }

    SvgStyle style_axis = SvgStyle().Stroke("black", 1.0f);
    // Draw X Axis
    svg_stream.DrawText(.05f * W, 1.2f * H, .1f * H, stringifier(range.first),
                        "black");
    svg_stream.DrawText(W, 1.2f * H, .1f * H, stringifier(range.second),
                        "black");
    svg_stream.DrawLine(0, 1.1f * H, W, 1.1f * H, style_axis);
    // Draw Y Axis
    svg_stream.DrawText(1.2f * W, .1f * H, .1f * H, stringifier(maxi), "black");
    svg_stream.DrawText(1.2f * W, H, .1f * H, "0", "black");
    svg_stream.DrawLine(1.1f * W, 0, 1.1f * W, H, style_axis);

    std::ofstream svg_filestream(filename.c_str());
    svg_filestream << svg_stream.CloseSvgFile().str();
  }
};

} // namespace DAGSfM

#endif