#include "graph/image_graph.h"

#include "graph/svg_drawer.h"
#include "graph/color_gradient.h"

namespace DAGSfM {

void ImageGraph::ExtractLargestCC() {
  graph::UnionFind uf(image_ids_.size());
  std::vector<size_t> tmp_nodes(image_ids_.begin(), image_ids_.end());
  uf.InitWithNodes(tmp_nodes);

  for (auto image_pair : image_pairs_) {
    uf.Union(image_pair.first, image_pair.second);
  }

  std::unordered_map<size_t, std::vector<image_t>> components;
  for (auto image_id : image_ids_) {
    const size_t parent_id = uf.FindRoot(image_id);
    components[parent_id].push_back(image_id);
  }

  size_t num_largest_component = 0;
  size_t largest_component_id;
  for (const auto& it : components) {
    if (num_largest_component < it.second.size()) {
      num_largest_component = it.second.size();
      largest_component_id = it.first;
    }
  }

  image_ids_.clear();
  image_ids_.assign(components[largest_component_id].begin(),
                    components[largest_component_id].end());
  image_ids_.shrink_to_fit();
  std::sort(image_ids_.begin(), image_ids_.end());

  LOG(INFO) << "There are " << components.size() << " connected components.";
  int num_small_connected_components = 0;
  for (auto component : components) {
    if (component.second.size() < image_ids_.size()) {
      num_small_connected_components++;
    } else {
      LOG(INFO) << "Component #" << component.first << "# has "
                << component.second.size() << " images.";
    }
  }
  LOG(INFO) << "There are " << num_small_connected_components
            << " small connected components are discarded.";
}

void ImageGraph::GetImagePairsSubset(
    const std::vector<image_t>& image_ids,
    std::vector<std::pair<image_t, image_t>>& image_pairs) {
  std::unordered_set<image_t> image_sets(image_ids.begin(), image_ids.end());
  GetImagePairsSubset(image_sets, image_pairs);
}

void ImageGraph::GetImagePairsSubset(
    const std::unordered_set<image_t>& image_ids,
    std::vector<std::pair<image_t, image_t>>& image_pairs) {
  for (auto image_pair : image_pairs_) {
    if (image_ids.count(image_pair.first) > 0 &&
        image_ids.count(image_pair.second) > 0) {
      image_pairs.push_back(image_pair);
    }
  }
}

void ImageGraph::OutputSVG(const std::string& filename) {
  ColorGradient heatmap_gradient(ColorGradient::K2BlueRedHeatMap());

  std::vector<int>::iterator max_score_itr =
                  std::max_element(scores_.begin(), scores_.end());
  const float max_score = std::max(1.0f, static_cast<float>(*max_score_itr));

  const float scale_factor = 5.0f;
  const size_t image_num = image_ids_.size();

  SvgDrawer svg_drawer((image_num + 3) * 5, (image_num + 3) * 5);

  // Draw rectangles for all image pairs.
  for (size_t k = 0; k < image_pairs_.size(); k++) {
    const auto& image_pair = image_pairs_[k];
    const int score = scores_[k];
    const image_t i = image_pair.first;
    const image_t j = image_pair.second;

    float r, g, b;
    heatmap_gradient.GetColor(score / max_score, &r, &g, &b);
    std::ostringstream os_color;
    os_color << "rgb(" << static_cast<int>(r * 255)
             << "," << static_cast<int>(g * 255) << "," << static_cast<int>(b * 255) << ")";

    std::ostringstream os_tooltip;
    os_tooltip << "(" << j << "," << i << " " << score << ")";
    svg_drawer.DrawSquare(j * scale_factor, i * scale_factor, scale_factor / 2.0f,
                          SvgStyle().Fill(os_color.str()).NoStroke().ToolTip(os_tooltip.str()));
    
    os_tooltip.clear();
    os_tooltip << "(" << i << "," << j << " " << score << ")";
    svg_drawer.DrawSquare(i * scale_factor, j * scale_factor, scale_factor / 2.0f,
                          SvgStyle().Fill(os_color.str()).NoStroke().ToolTip(os_tooltip.str()));
  }

  // Display axes with 0-> image_num annotation
  std::ostringstream os_num_images;
  os_num_images << image_num;
  svg_drawer.DrawText((image_num + 1) * scale_factor, scale_factor, scale_factor, "0", "black");
  svg_drawer.DrawText((image_num + 1) * scale_factor, (image_num - 1) * scale_factor,
                       scale_factor, os_num_images.str(), "black");
  svg_drawer.DrawLine((image_num + 1) * scale_factor, 2 * scale_factor,
                      (image_num + 1) * scale_factor, (image_num - 2) * scale_factor,
                      SvgStyle().Stroke("black", 1.0));
  svg_drawer.DrawText(scale_factor, (image_num + 1) * scale_factor, scale_factor, "0", "black");
  svg_drawer.DrawText((image_num - 1) * scale_factor, (image_num + 1) * scale_factor,
                      scale_factor, os_num_images.str(), "black");
  svg_drawer.DrawLine(2 * scale_factor, (image_num + 1) * scale_factor,
                      (image_num - 2) * scale_factor, (image_num + 1) * scale_factor,
                      SvgStyle().Stroke("black", 1.0));
  
  std::ofstream svg_ofs(filename);
  svg_ofs << svg_drawer.CloseSvgFile().str();
}

}  // namespace DAGSfM