#pragma once

#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/srv/area_coverage.hpp"
#include "open_mower_next/srv/polygon_coverage.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fields2cover.h>
#include <vector>

namespace open_mower_next::coverage_server
{

class CoverageServerNode : public rclcpp::Node
{
public:
  explicit CoverageServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CoverageServerNode();

private:
  double robot_width_;         // Width of the robot platform
  double operation_width_;     // Width of the mowing tool/blade
  double min_turning_radius_;  // Minimum turning radius of the robot
  double headland_connector_step_;
  double headland_transition_lookahead_;
  double headland_connector_tangent_length_;

  rclcpp::Service<open_mower_next::srv::AreaCoverage>::SharedPtr area_coverage_service_;

  rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr map_subscription_;
  open_mower_next::msg::Map current_map_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void handleAreaCoverageRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
    std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response);

  void mapCallback(const open_mower_next::msg::Map::SharedPtr msg);

  uint16_t generateCoveragePath(
    uint16_t headland_loops, uint16_t swath_angle,
    const geometry_msgs::msg::PolygonStamped & field_polygon,
    const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
    nav_msgs::msg::Path & response_paths, std::string & message);

  bool findExclusionsInPolygon(
    const geometry_msgs::msg::PolygonStamped & field_polygon,
    std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons, std::string & message);
  size_t appendHeadlandSwaths(
    f2c::types::Swaths & swaths, const f2c::types::Cells & headland_cells, double swath_width);
  size_t appendObstacleHeadlandSwaths(
    f2c::types::Swaths & swaths,
    const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
    uint16_t headland_loops, double swath_width);
  f2c::types::Swaths generateSwaths(
    const f2c::types::Robot & robot, const f2c::types::Cells & headland_cells,
    const f2c::types::Cells & coverage_cells,
    const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
    uint16_t headland_loops, uint16_t swath_angle);
  std::vector<f2c::types::Swath> orderHeadlandSwaths(
    const f2c::types::Swaths & swaths, uint16_t headland_loops);
  f2c::types::Swath closeHeadlandSwath(const f2c::types::Swath & swath) const;
  f2c::types::Swath densifySwath(const f2c::types::Swath & swath) const;
  f2c::types::Swath normalizeHeadlandSwath(const f2c::types::Swath & swath, bool clockwise) const;
  f2c::types::Swath rotateClosedSwathToNearestPoint(
    const f2c::types::Swath & swath, const f2c::types::Point & reference,
    bool advance_start_point, bool clockwise);
  double signedArea(const f2c::types::Swath & swath) const;
  double nearestConnectDistance(
    const f2c::types::Swath & swath, const f2c::types::Point & reference) const;
  double pointDistance(const f2c::types::Point & a, const f2c::types::Point & b) const;
  nav_msgs::msg::Path createConnectedHeadlandPath(
    const f2c::types::Swaths & swaths, const std::string & frame_id, uint16_t headland_loops);
  void appendSwathToPath(
    nav_msgs::msg::Path & path, const f2c::types::Swath & swath, const std::string & frame_id);
  void appendConnectorToPath(
    nav_msgs::msg::Path & path, const f2c::types::Swath & start_swath,
    const f2c::types::Swath & end_swath, const std::string & frame_id);
  std::vector<f2c::types::Point> createSmoothConnector(
    const f2c::types::Swath & start_swath, const f2c::types::Swath & end_swath) const;

  msg::Area::SharedPtr findAreaById(const std::string & area_id);

  std::vector<std::string> findAreasInPolygon(const geometry_msgs::msg::PolygonStamped & polygon);

  visualization_msgs::msg::MarkerArray createVisualizationMarkers(
    const f2c::types::Swaths & swaths, const std::string & frame_id, uint16_t headland_loops);
};

}  // namespace open_mower_next::coverage_server
