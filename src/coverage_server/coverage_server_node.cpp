#include "coverage_server_node.hpp"

#include "utils.h"

#include <tf2/LinearMath/Quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>

namespace open_mower_next::coverage_server
{
CoverageServerNode::CoverageServerNode(const rclcpp::NodeOptions & options)
: Node("coverage_server", options)
{
  RCLCPP_INFO(get_logger(), "Starting Coverage Server node");

  robot_width_ = declare_parameter("robot_width", 0.325);
  operation_width_ = declare_parameter("operation_width", 0.065);
  min_turning_radius_ = declare_parameter("min_turning_radius", 0.01);
  headland_connector_step_ = declare_parameter("headland_connector_step", 0.05);
  headland_transition_lookahead_ = declare_parameter("headland_transition_lookahead", 0.5);
  headland_connector_tangent_length_ = declare_parameter("headland_connector_tangent_length", 0.6);

  RCLCPP_INFO(
    get_logger(), "Configured with robot_width=%.3f, mowing_tool_width=%.3f", robot_width_,
    operation_width_);

  area_coverage_service_ = create_service<open_mower_next::srv::AreaCoverage>(
    "area_coverage", std::bind(
                       &CoverageServerNode::handleAreaCoverageRequest, this, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3));

  // Setup map subscription
  map_subscription_ = create_subscription<open_mower_next::msg::Map>(
    "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&CoverageServerNode::mapCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "coverage/path", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
  visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "coverage/visualization", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

  RCLCPP_INFO(get_logger(), "Coverage Server initialized and ready");
}

CoverageServerNode::~CoverageServerNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down Coverage Server node");
}

void CoverageServerNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  current_map_ = *msg;
  RCLCPP_INFO(get_logger(), "Received updated map with %zu areas", msg->areas.size());
}

void CoverageServerNode::handleAreaCoverageRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
  std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response)
{
  (void)request_header;

  if (request->area_id.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area ID cannot be empty";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received area coverage request for area ID: %s", request->area_id.c_str());

  const auto area = findAreaById(request->area_id);

  if (area == nullptr) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area not found";
    return;
  }

  if (area.get()->type != open_mower_next::msg::Area::TYPE_OPERATION) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area is not of type OPERATION";
    return;
  }

  const auto & area_polygon = area.get()->area;

  if (!utils::isValid(area_polygon.polygon)) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area has an invalid polygon";
    return;
  }

  // Find exclusions if requested
  std::vector<geometry_msgs::msg::PolygonStamped> exclusion_polygons;
  if (request->with_exclusions) {
    std::string exclusion_message;
    if (!findExclusionsInPolygon(area_polygon, exclusion_polygons, exclusion_message)) {
      response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_EXCLUSION;
      response->message = exclusion_message;
      return;
    }
  }

  const auto coverage_cells = utils::toCells(area_polygon, exclusion_polygons);
  if (coverage_cells.isEmpty() || coverage_cells.size() == 0) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_EXCLUSION;
    response->message = "Exclusions remove the entire requested area";
    return;
  }

  const auto max_curvature = min_turning_radius_ > 0.0 ? 1.0 / min_turning_radius_ : 1.0;
  f2c::types::Robot robot(robot_width_, operation_width_, max_curvature);
  const auto headland_cells = utils::toCells(area_polygon, {});

  const auto swaths =
    generateSwaths(
      robot, headland_cells, coverage_cells, exclusion_polygons, request->headland_loops,
      request->swath_angle);

  nav_msgs::msg::Path path =
    createConnectedHeadlandPath(swaths, area_polygon.header.frame_id, request->headland_loops);

  response->message = "Coverage path generated successfully";
  response->code = open_mower_next::srv::AreaCoverage::Response::CODE_SUCCESS;
  response->path = path;
  response->coverage_geometry = utils::toMsg(coverage_cells, area_polygon.header.frame_id);
  response->area_id = request->area_id;

  path_pub_->publish(path);
  const auto markers =
    createVisualizationMarkers(swaths, area_polygon.header.frame_id, request->headland_loops);
  visualization_pub_->publish(markers);

  RCLCPP_INFO(
    get_logger(),
    "Generated connected headland path with %zu poses from %zu planned swaths for area ID: %s",
    path.poses.size(), swaths.size(), request->area_id.c_str());
}

msg::Area::SharedPtr CoverageServerNode::findAreaById(const std::string & area_id)
{
  geometry_msgs::msg::PolygonStamped result;
  result.header.frame_id = "map";
  result.header.stamp = now();

  for (const auto & area : current_map_.areas) {
    if (area.id == area_id) {
      RCLCPP_INFO(
        get_logger(), "Found area with ID: %s, name: %s", area_id.c_str(), area.name.c_str());

      return std::make_shared<msg::Area>(area);
    }
  }

  return nullptr;
}

std::vector<std::string> CoverageServerNode::findAreasInPolygon(
  const geometry_msgs::msg::PolygonStamped & polygon)
{
  (void)polygon;

  std::vector<std::string> area_ids;

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_OPERATION) {
      // TODO: Implement proper polygon intersection test
      area_ids.push_back(area.id);
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu operation areas within the polygon", area_ids.size());
  return area_ids;
}

bool CoverageServerNode::findExclusionsInPolygon(
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons, std::string & message)
{
  exclusion_polygons.clear();
  message.clear();

  const auto field_cell = utils::toCell(field_polygon);

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_EXCLUSION) {
      if (!utils::isValid(area.area.polygon)) {
        message = "Exclusion area " + area.id + " has an invalid polygon";
        return false;
      }

      geometry_msgs::msg::PolygonStamped exclusion;
      exclusion.header = field_polygon.header;
      exclusion.polygon = area.area.polygon;

      const auto exclusion_cell = utils::toCell(exclusion);
      if (field_cell.disjoint(exclusion_cell)) {
        RCLCPP_INFO(
          get_logger(), "Skipping exclusion area outside field: %s, name: %s", area.id.c_str(),
          area.name.c_str());
        continue;
      }

      exclusion_polygons.push_back(exclusion);
      RCLCPP_INFO(
        get_logger(), "Applying exclusion area with ID: %s, name: %s", area.id.c_str(),
        area.name.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu exclusion areas", exclusion_polygons.size());
  return true;
}

size_t CoverageServerNode::appendHeadlandSwaths(
  f2c::types::Swaths & swaths, const f2c::types::Cells & headland_cells, const double swath_width)
{
  size_t appended_count = 0;

  if (headland_cells.isEmpty() || headland_cells.size() == 0) {
    return appended_count;
  }

  for (size_t cell_index = 0; cell_index < headland_cells.size(); ++cell_index) {
    const auto cell = headland_cells.getGeometry(cell_index);
    if (cell.isEmpty() || cell.size() == 0) {
      continue;
    }

    for (size_t ring_index = 0; ring_index < cell.size(); ++ring_index) {
      const auto ring = cell.getGeometry(ring_index);
      if (ring.isEmpty() || ring.size() < 2) {
        continue;
      }

      swaths.append(
        f2c::types::LineString(ring), swath_width, f2c::types::SwathType::HEADLAND);
      ++appended_count;
    }
  }

  return appended_count;
}

size_t CoverageServerNode::appendObstacleHeadlandSwaths(
  f2c::types::Swaths & swaths,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
  const uint16_t headland_loops, const double swath_width)
{
  size_t appended_count = 0;
  if (exclusion_polygons.empty() || headland_loops == 0) {
    return appended_count;
  }

  for (const auto & exclusion_polygon : exclusion_polygons) {
    const auto exclusion_cell = utils::toCell(exclusion_polygon);
    if (exclusion_cell.isEmpty() || exclusion_cell.size() == 0) {
      continue;
    }
    const auto obstacle_base = exclusion_cell.convexHull();

    for (uint16_t loop_index = 0; loop_index < headland_loops; ++loop_index) {
      const auto offset = swath_width * (static_cast<double>(loop_index) + 0.5);
      const auto obstacle_headland = f2c::types::Cell::buffer(obstacle_base, offset);
      if (obstacle_headland.isEmpty() || obstacle_headland.size() == 0) {
        continue;
      }

      const auto ring = obstacle_headland.getGeometry(0);
      if (ring.isEmpty() || ring.size() < 2) {
        continue;
      }

      swaths.append(
        f2c::types::LineString(ring), swath_width, f2c::types::SwathType::HEADLAND);
      ++appended_count;
    }
  }

  return appended_count;
}

f2c::types::Swaths CoverageServerNode::generateSwaths(
  const f2c::types::Robot & robot, const f2c::types::Cells & headland_cells,
  const f2c::types::Cells & coverage_cells,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
  const uint16_t headland_loops, const uint16_t swath_angle)
{
  f2c::hg::ConstHL hg;

  RCLCPP_INFO(get_logger(), "Generating swaths...");

  f2c::types::Swaths planned_swaths;
  auto mainland = coverage_cells;

  if (headland_loops > 0) {
    const auto headland_width = robot.getCovWidth() * headland_loops;
    const auto headland_rings =
      hg.generateHeadlandSwaths(headland_cells, robot.getCovWidth(), headland_loops, true);
    std::vector<std::vector<f2c::types::Swath>> headland_groups;

    RCLCPP_INFO(
      get_logger(), "Generating %d headland loops with %.3f m spacing", headland_loops,
      robot.getCovWidth());

    for (size_t loop_index = 0; loop_index < headland_rings.size(); ++loop_index) {
      f2c::types::Swaths loop_swaths;
      appendHeadlandSwaths(loop_swaths, headland_rings[loop_index], robot.getCovWidth());

      std::vector<f2c::types::Swath> remaining;
      for (const auto & swath : loop_swaths) {
        remaining.push_back(swath);
      }

      if (loop_index == 0 || headland_groups.empty()) {
        for (const auto & swath : remaining) {
          headland_groups.push_back({swath});
        }
        continue;
      }

      std::vector<bool> used(remaining.size(), false);
      for (auto & group : headland_groups) {
        if (group.empty()) {
          continue;
        }

        size_t best_index = remaining.size();
        double best_distance = std::numeric_limits<double>::max();
        const auto reference = group.back().getPoint(0);
        for (size_t index = 0; index < remaining.size(); ++index) {
          if (used[index]) {
            continue;
          }
          const auto distance = nearestConnectDistance(remaining[index], reference);
          if (distance < best_distance) {
            best_distance = distance;
            best_index = index;
          }
        }

        if (best_index < remaining.size()) {
          group.push_back(
            rotateClosedSwathToNearestPoint(remaining[best_index], reference, false, false));
          used[best_index] = true;
        }
      }

      for (size_t index = 0; index < remaining.size(); ++index) {
        if (!used[index]) {
          headland_groups.push_back({remaining[index]});
        }
      }
    }

    size_t headland_swath_count = 0;
    for (const auto & group : headland_groups) {
      for (const auto & swath : group) {
        planned_swaths.push_back(swath);
        ++headland_swath_count;
      }
    }

    RCLCPP_INFO(
      get_logger(), "Generated %zu headland swaths in %zu contour groups", headland_swath_count,
      headland_groups.size());

    const auto obstacle_headland_count = appendObstacleHeadlandSwaths(
      planned_swaths, exclusion_polygons, headland_loops, robot.getCovWidth());
    if (obstacle_headland_count > 0) {
      RCLCPP_INFO(
        get_logger(), "Generated %zu obstacle headland swaths around %zu exclusions",
        obstacle_headland_count, exclusion_polygons.size());
    }

    mainland = hg.generateHeadlands(coverage_cells, headland_width);
  }

  if (mainland.isEmpty() || mainland.size() == 0) {
    return f2c::types::Swaths{};
  }

  f2c::sg::BruteForce swath_generator;
  f2c::obj::SwathLength swath_objective;
  const f2c::rp::BoustrophedonOrder sorter;

  for (size_t i = 0; i < mainland.size(); ++i) {
    const auto cell = mainland.getGeometry(i);
    if (cell.isEmpty() || cell.area() <= 0.0) {
      continue;
    }

    const auto best_angle =
      swath_generator.computeBestAngle(swath_objective, robot.getCovWidth(), cell);
    const auto requested_angle = best_angle + (swath_angle * M_PI / 180.0);

    const auto swaths = swath_generator.generateSwaths(requested_angle, robot.getCovWidth(), cell);
    planned_swaths.append(sorter.genSortedSwaths(swaths));
  }

  RCLCPP_INFO(get_logger(), "Generated %zu total swaths", planned_swaths.size());

  return planned_swaths;
}

nav_msgs::msg::Path CoverageServerNode::createConnectedHeadlandPath(
  const f2c::types::Swaths & swaths, const std::string & frame_id,
  const uint16_t headland_loops)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;

  bool has_previous_headland = false;
  f2c::types::Swath previous_headland;
  size_t headland_count = 0;
  const auto ordered_headlands = orderHeadlandSwaths(swaths, headland_loops);

  for (const auto & swath : ordered_headlands) {
    const auto starts_new_headland_group =
      headland_loops > 0 && headland_count % static_cast<size_t>(headland_loops) == 0;
    if (has_previous_headland && !starts_new_headland_group) {
      appendConnectorToPath(path, previous_headland, swath, frame_id);
    }

    appendSwathToPath(path, swath, frame_id);
    previous_headland = swath;
    has_previous_headland = true;
    ++headland_count;
  }

  RCLCPP_INFO(get_logger(), "Connected %zu headland swaths for executable coverage path", headland_count);

  return path;
}

std::vector<f2c::types::Swath> CoverageServerNode::orderHeadlandSwaths(
  const f2c::types::Swaths & swaths, const uint16_t headland_loops)
{
  if (headland_loops == 0) {
    return {};
  }

  std::vector<std::vector<f2c::types::Swath>> groups;
  size_t headland_index = 0;
  for (const auto & swath : swaths) {
    if (swath.getType() == f2c::types::SwathType::HEADLAND && swath.numPoints() >= 2) {
      const auto group_number =
        headland_loops > 0 ? headland_index / static_cast<size_t>(headland_loops) : 0;
      const auto clockwise = group_number > 0;
      if (headland_index % static_cast<size_t>(headland_loops) == 0) {
        groups.emplace_back();
      }
      groups.back().push_back(normalizeHeadlandSwath(swath, clockwise));
      ++headland_index;
    }
  }

  std::vector<f2c::types::Swath> ordered;
  if (groups.empty()) {
    return ordered;
  }

  auto append_group = [&](std::vector<f2c::types::Swath> group, const bool clockwise) {
    if (group.empty()) {
      return;
    }

    if (ordered.empty()) {
      ordered.push_back(group.front());
    } else {
      const auto reference = ordered.back().getPoint(0);
      ordered.push_back(rotateClosedSwathToNearestPoint(group.front(), reference, false, clockwise));
    }

    for (size_t group_index = 1; group_index < group.size(); ++group_index) {
      const auto reference = ordered.back().getPoint(0);
      const auto advance_start_point = group_index >= 2;
      ordered.push_back(
        rotateClosedSwathToNearestPoint(group[group_index], reference, advance_start_point, clockwise));
    }
  };

  append_group(groups.front(), false);
  groups.erase(groups.begin());

  while (!groups.empty()) {
    const auto reference = ordered.back().getPoint(0);
    size_t best_group_index = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (size_t group_index = 0; group_index < groups.size(); ++group_index) {
      if (groups[group_index].empty()) {
        continue;
      }
      const auto distance = nearestConnectDistance(groups[group_index].front(), reference);
      if (distance < best_distance) {
        best_distance = distance;
        best_group_index = group_index;
      }
    }

    auto group = groups[best_group_index];
    groups.erase(groups.begin() + static_cast<long>(best_group_index));
    append_group(group, true);
  }

  return ordered;
}

f2c::types::Swath CoverageServerNode::closeHeadlandSwath(const f2c::types::Swath & swath) const
{
  if (swath.numPoints() < 2) {
    return swath;
  }

  std::vector<f2c::types::Point> points;
  points.reserve(swath.numPoints() + 1);
  for (size_t index = 0; index < swath.numPoints(); ++index) {
    points.push_back(swath.getPoint(index));
  }

  if (pointDistance(points.front(), points.back()) > 1e-6) {
    points.push_back(points.front());
  }

  return f2c::types::Swath(
    f2c::types::LineString(points), swath.getWidth(), swath.getId(), swath.getType());
}

f2c::types::Swath CoverageServerNode::densifySwath(const f2c::types::Swath & swath) const
{
  if (swath.numPoints() < 2) {
    return swath;
  }

  const auto max_segment_length = std::max(swath.getWidth(), 0.01);
  std::vector<f2c::types::Point> points;
  points.reserve(swath.numPoints());

  for (size_t index = 0; index + 1 < swath.numPoints(); ++index) {
    const auto start = swath.getPoint(index);
    const auto end = swath.getPoint(index + 1);
    const auto distance = pointDistance(start, end);
    const auto steps =
      std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / max_segment_length)));

    for (size_t step = 0; step < steps; ++step) {
      const auto t = static_cast<double>(step) / static_cast<double>(steps);
      const auto x = start.getX() + (end.getX() - start.getX()) * t;
      const auto y = start.getY() + (end.getY() - start.getY()) * t;
      points.emplace_back(x, y);
    }
  }
  points.push_back(swath.getPoint(swath.numPoints() - 1));

  return f2c::types::Swath(
    f2c::types::LineString(points), swath.getWidth(), swath.getId(), swath.getType());
}

f2c::types::Swath CoverageServerNode::normalizeHeadlandSwath(
  const f2c::types::Swath & swath, const bool clockwise) const
{
  auto output = densifySwath(closeHeadlandSwath(swath));
  const auto is_clockwise = signedArea(output) < 0.0;
  if (is_clockwise != clockwise) {
    output.reverse();
    output = densifySwath(closeHeadlandSwath(output));
  }
  return output;
}

f2c::types::Swath CoverageServerNode::rotateClosedSwathToNearestPoint(
  const f2c::types::Swath & swath, const f2c::types::Point & reference,
  const bool advance_start_point, const bool clockwise)
{
  const auto normalized_swath = normalizeHeadlandSwath(swath, clockwise);

  if (swath.numPoints() < 3) {
    auto output = normalized_swath;
    if (pointDistance(output.getPoint(output.numPoints() - 1), reference) <
        pointDistance(output.getPoint(0), reference)) {
      output.reverse();
    }
    return output;
  }

  const auto has_duplicate_closure =
    pointDistance(
      normalized_swath.getPoint(0), normalized_swath.getPoint(normalized_swath.numPoints() - 1)) <=
    1e-6;
  const auto unique_point_count =
    normalized_swath.numPoints() - (has_duplicate_closure ? 1 : 0);

  size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (size_t index = 0; index < unique_point_count; ++index) {
    const auto point = normalized_swath.getPoint(index);
    const auto distance = pointDistance(point, reference);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  auto wrap_index = [unique_point_count](const int value) {
    const auto count = static_cast<int>(unique_point_count);
    return static_cast<size_t>((value % count + count) % count);
  };

  auto build_candidate = [&](const int direction) {
    auto start_index = static_cast<int>(nearest_index);
    if (advance_start_point && unique_point_count > 1) {
      start_index = static_cast<int>(wrap_index(start_index + direction));
    }

    std::vector<f2c::types::Point> candidate_points;
    candidate_points.reserve(swath.numPoints());
    for (size_t offset = 0; offset < unique_point_count; ++offset) {
      const auto index = wrap_index(start_index + direction * static_cast<int>(offset));
      candidate_points.push_back(normalized_swath.getPoint(index));
    }
    candidate_points.push_back(candidate_points.front());

    return f2c::types::Swath(
      f2c::types::LineString(candidate_points), normalized_swath.getWidth(),
      normalized_swath.getId(), normalized_swath.getType());
  };

  return build_candidate(1);
}

double CoverageServerNode::signedArea(const f2c::types::Swath & swath) const
{
  if (swath.numPoints() < 3) {
    return 0.0;
  }

  double area = 0.0;
  for (size_t index = 0; index + 1 < swath.numPoints(); ++index) {
    const auto current = swath.getPoint(index);
    const auto next = swath.getPoint(index + 1);
    area += current.getX() * next.getY() - next.getX() * current.getY();
  }

  return area * 0.5;
}

double CoverageServerNode::nearestConnectDistance(
  const f2c::types::Swath & swath, const f2c::types::Point & reference) const
{
  if (swath.numPoints() < 3) {
    return std::min(
      pointDistance(swath.getPoint(0), reference),
      pointDistance(swath.getPoint(swath.numPoints() - 1), reference));
  }

  double nearest_distance = std::numeric_limits<double>::max();
  const auto has_duplicate_closure =
    pointDistance(swath.getPoint(0), swath.getPoint(swath.numPoints() - 1)) <= 1e-6;
  const auto unique_point_count = swath.numPoints() - (has_duplicate_closure ? 1 : 0);
  for (size_t index = 0; index < unique_point_count; ++index) {
    nearest_distance = std::min(nearest_distance, pointDistance(swath.getPoint(index), reference));
  }
  return nearest_distance;
}

double CoverageServerNode::pointDistance(
  const f2c::types::Point & a, const f2c::types::Point & b) const
{
  return std::hypot(a.getX() - b.getX(), a.getY() - b.getY());
}

void CoverageServerNode::appendSwathToPath(
  nav_msgs::msg::Path & path, const f2c::types::Swath & swath, const std::string & frame_id)
{
  const auto is_headland = swath.getType() == f2c::types::SwathType::HEADLAND;
  const auto closes_to_start =
    swath.numPoints() >= 2 &&
    pointDistance(swath.getPoint(0), swath.getPoint(swath.numPoints() - 1)) <= 1e-6;

  for (size_t point_index = 0; point_index < swath.numPoints(); ++point_index) {
    const auto point = swath.getPoint(point_index);
    double yaw = swath.getOutAngle();

    if (point_index + 1 < swath.numPoints()) {
      const auto next_point = swath.getPoint(point_index + 1);
      yaw = std::atan2(next_point.getY() - point.getY(), next_point.getX() - point.getX());
    } else if (is_headland && !closes_to_start) {
      const auto next_point = swath.getPoint(0);
      yaw = std::atan2(next_point.getY() - point.getY(), next_point.getX() - point.getX());
    }

    path.poses.push_back(utils::toMsg(point.getX(), point.getY(), yaw, frame_id));
  }

  if (is_headland && !closes_to_start) {
    const auto point = swath.getPoint(0);
    path.poses.push_back(utils::toMsg(point.getX(), point.getY(), swath.getInAngle(), frame_id));
  }
}

void CoverageServerNode::appendConnectorToPath(
  nav_msgs::msg::Path & path, const f2c::types::Swath & start_swath,
  const f2c::types::Swath & end_swath, const std::string & frame_id)
{
  const auto points = createSmoothConnector(start_swath, end_swath);

  for (size_t index = 1; index < points.size(); ++index) {
    const auto & point = points[index];
    double yaw = end_swath.getInAngle();

    if (index + 1 < points.size()) {
      const auto & next_point = points[index + 1];
      yaw = std::atan2(next_point.getY() - point.getY(), next_point.getX() - point.getX());
    }

    path.poses.push_back(utils::toMsg(point.getX(), point.getY(), yaw, frame_id));
  }
}

std::vector<f2c::types::Point> CoverageServerNode::createSmoothConnector(
  const f2c::types::Swath & start_swath, const f2c::types::Swath & end_swath) const
{
  const auto start = start_swath.getPoint(0);
  const auto end = end_swath.getPoint(0);
  const auto dx = end.getX() - start.getX();
  const auto dy = end.getY() - start.getY();
  const auto distance = std::hypot(dx, dy);
  if (distance <= 1e-6) {
    return {start, end};
  }

  const auto steps = std::max<size_t>(
    2, static_cast<size_t>(std::ceil(distance / std::max(headland_connector_step_, 0.01))));

  std::vector<f2c::types::Point> points;
  points.reserve(steps + 1);
  for (size_t step = 0; step <= steps; ++step) {
    const auto t = static_cast<double>(step) / static_cast<double>(steps);
    const auto x = start.getX() + dx * t;
    const auto y = start.getY() + dy * t;
    points.emplace_back(x, y);
  }

  return points;
}

visualization_msgs::msg::MarkerArray CoverageServerNode::createVisualizationMarkers(
  const f2c::types::Swaths & swaths, const std::string & frame_id,
  const uint16_t headland_loops)
{
  visualization_msgs::msg::MarkerArray markers;

  // Add a deletion marker to clear all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = frame_id;
  delete_marker.header.stamp = now();
  delete_marker.ns = "all";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  bool has_previous_headland = false;
  f2c::types::Swath previous_headland;
  int marker_id = 0;
  size_t headland_index = 0;
  const auto ordered_headlands = orderHeadlandSwaths(swaths, headland_loops);

  for (const auto & swath : ordered_headlands) {
    visualization_msgs::msg::Marker marker;

    const auto starts_new_headland_group =
      headland_loops > 0 && headland_index % static_cast<size_t>(headland_loops) == 0;
    if (has_previous_headland && !starts_new_headland_group) {
      visualization_msgs::msg::Marker connector_marker;
      connector_marker.ns = "headland_connector";
      connector_marker.color.r = 0.0f;
      connector_marker.color.g = 1.0f;
      connector_marker.color.b = 0.0f;
      connector_marker.color.a = 1.0f;
      connector_marker.header.frame_id = frame_id;
      connector_marker.header.stamp = now();
      connector_marker.id = marker_id++;
      connector_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      connector_marker.action = visualization_msgs::msg::Marker::ADD;
      connector_marker.scale.x = swath.getWidth();
      for (const auto & point : createSmoothConnector(previous_headland, swath)) {
        connector_marker.points.push_back(utils::toMsg(point));
      }
      markers.markers.push_back(connector_marker);
    }

    marker.ns = "headland";
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = swath.getWidth();

    for (size_t j = 0; j < swath.numPoints(); ++j) {
      const auto point = swath.getPoint(j);
      marker.points.push_back(utils::toMsg(point));
    }
    if (
      swath.numPoints() >= 2 &&
      pointDistance(swath.getPoint(0), swath.getPoint(swath.numPoints() - 1)) > 1e-6) {
      marker.points.push_back(utils::toMsg(swath.getPoint(0)));
    }
    markers.markers.push_back(marker);

    previous_headland = swath;
    has_previous_headland = true;
    ++headland_index;
  }

  for (const auto & swath : swaths) {
    if (swath.getType() == f2c::types::SwathType::HEADLAND) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.ns = "mainland";
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = swath.getWidth();

    for (size_t j = 0; j < swath.numPoints(); ++j) {
      const auto point = swath.getPoint(j);
      marker.points.push_back(utils::toMsg(point));
    }

    markers.markers.push_back(marker);
  }

  return markers;
}

}  // namespace open_mower_next::coverage_server
