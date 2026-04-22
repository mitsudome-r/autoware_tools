// Copyright 2026 Autoware Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Configuration.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("opendrive_to_lanelet2");

  const auto xodr_input_path = node->declare_parameter<std::string>("xodr_input_path");
  const auto llt_output_path = node->declare_parameter<std::string>("llt_output_path");
  const auto sample_step_m = node->declare_parameter<double>("sample_step_m", 1.0);
  const auto merge_tol_m = node->declare_parameter<double>("merge_tol_m", 0.05);
  const auto skip_sidewalks = node->declare_parameter<bool>("skip_sidewalks", false);
  const auto traffic_light_types =
    node->declare_parameter<std::string>("traffic_light_types", "");
  const auto origin_lat = node->declare_parameter<double>("origin_lat", 0.0);
  const auto origin_lon = node->declare_parameter<double>("origin_lon", 0.0);

  lanelet::io::Configuration params;
  params["autoware_opendrive/sample_step_m"] = sample_step_m;
  params["autoware_opendrive/merge_tol_m"] = merge_tol_m;
  params["autoware_opendrive/skip_sidewalks"] = skip_sidewalks;
  if (!traffic_light_types.empty()) {
    params["autoware_opendrive/traffic_light_types"] = traffic_light_types;
  }

  lanelet::projection::UtmProjector projector{lanelet::Origin({origin_lat, origin_lon})};
  lanelet::ErrorMessages errors;

  auto map =
    lanelet::load(xodr_input_path, "autoware_opendrive_handler", projector, &errors, params);

  for (const auto & error : errors) {
    RCLCPP_WARN_STREAM(node->get_logger(), error);
  }
  if (!map) {
    RCLCPP_ERROR_STREAM(
      node->get_logger(), "failed to parse OpenDRIVE map: " << xodr_input_path);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  std::cout << "Loaded OpenDRIVE map: " << xodr_input_path << std::endl;
  std::cout << "  points: " << map->pointLayer.size() << std::endl;
  std::cout << "  lanelets: " << map->laneletLayer.size() << std::endl;

  // Preserve the OpenDRIVE-local XY as local_x/local_y tags so they survive
  // the lat/lon round-trip through the UTM projector on write/read.
  for (lanelet::Point3d & pt : map->pointLayer) {
    pt.attributes()["local_x"] = pt.x();
    pt.attributes()["local_y"] = pt.y();
  }

  errors.clear();
  lanelet::write(llt_output_path, *map, projector, &errors);
  for (const auto & error : errors) {
    RCLCPP_WARN_STREAM(node->get_logger(), error);
  }
  std::cout << "Wrote Lanelet2 map: " << llt_output_path << std::endl;

  rclcpp::shutdown();
  return 0;
}
