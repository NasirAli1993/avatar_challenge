/*
 * =============================================================================
 * xArm7 Trajectory Drawing Node
 *
 * Function: Load geometric shapes (polygon, arc, B-spline) from JSON config,
 *           plan Cartesian paths using MoveIt2, control robot movement,
 *           and visualize trajectories in RViz.
 *
 * Revision: Modified for custom submission
 * =============================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/parameter_client.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <functional>

using json = nlohmann::json;
using namespace std::chrono_literals;

/**
 * @brief Generate clamped B-spline curve from control points
 * @param control_pts Input control points for the spline
 * @param spline_degree Degree of the spline
 * @param sample_num Number of sampled output points
 * @return Vector of 2D points on the spline curve
 */
std::vector<Eigen::Vector2d> generate_bspline_curve(
    const std::vector<Eigen::Vector2d>& control_pts,
    int spline_degree,
    int sample_num
)
{
    std::vector<Eigen::Vector2d> curve_output;
    int n = control_pts.size() - 1;
    int k = spline_degree;

    std::vector<double> knots(n + k + 2);
    for (int i = 0; i <= n + k + 1; ++i) {
        if (i <= k)
            knots[i] = 0.0;
        else if (i >= n + 1)
            knots[i] = 1.0;
        else
            knots[i] = static_cast<double>(i - k) / (n - k + 1);
    }

    std::function<double(int, int, double)> basis_func = [&](int i, int k, double t) -> double {
        if (k == 0)
            return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;

        double denom1 = knots[i + k] - knots[i];
        double term1 = 0.0;
        if (denom1 != 0) {
            term1 = (t - knots[i]) / denom1 * basis_func(i, k - 1, t);
        }

        double denom2 = knots[i + k + 1] - knots[i + 1];
        double term2 = 0.0;
        if (denom2 != 0) {
            term2 = (knots[i + k + 1] - t) / denom2 * basis_func(i + 1, k - 1, t);
        }

        return term1 + term2;
    };

    for (int step = 0; step < sample_num; ++step) {
        double t = static_cast<double>(step) / (sample_num - 1);
        Eigen::Vector2d pt(0.0, 0.0);
        for (int i = 0; i <= n; ++i) {
            pt += control_pts[i] * basis_func(i, k, t);
        }
        curve_output.push_back(pt);
    }

    return curve_output;
}

/**
 * @brief Transform a local 2D point into a 3D world pose
 * @param local_pt 2D point in local frame
 * @param base_frame Origin pose of the local frame
 * @return 3D pose in world frame
 */
geometry_msgs::msg::Pose transform_2d_to_3d_pose(
    const std::pair<double, double>& local_pt,
    const geometry_msgs::msg::Pose& base_frame
)
{
    geometry_msgs::msg::Pose world_pose;

    tf2::Quaternion quat;
    tf2::fromMsg(base_frame.orientation, quat);

    tf2::Vector3 local_vec(local_pt.first, local_pt.second, 0.0);
    tf2::Vector3 rotated_vec = tf2::quatRotate(quat, local_vec);

    world_pose.position.x = base_frame.position.x + rotated_vec.x();
    world_pose.position.y = base_frame.position.y + rotated_vec.y();
    world_pose.position.z = base_frame.position.z + rotated_vec.z();
    world_pose.orientation = base_frame.orientation;

    return world_pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("shape_drawer_node");

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker_array", qos_profile);

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "move_group");
    while (!param_client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Waiting for move_group parameter service...");
    }

    try {
        auto robot_desc = param_client->get_parameter<std::string>("robot_description");
        auto semantic_desc = param_client->get_parameter<std::string>("robot_description_semantic");

        node->declare_parameter("robot_description", robot_desc);
        node->declare_parameter("robot_description_semantic", semantic_desc);

        RCLCPP_INFO(node->get_logger(), "Robot model parameters loaded successfully.");
    }
    catch (const std::exception& err) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load robot model: %s", err.what());
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Shape drawing node started.");

    std::string config_path = "/home/dev/dev_ws/src/avatar_challenge/config/shapes.json";
    std::ifstream file_stream(config_path);
    json config_json;
    file_stream >> config_json;
    RCLCPP_INFO(node->get_logger(), "Successfully loaded JSON configuration.");

    const std::string PLANNING_GROUP = "xarm7";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    int marker_idx = 0;

    for (const auto& shape : config_json["shapes"])
    {
        auto pos = shape["start_pose"]["position"];
        auto rpy = shape["start_pose"]["orientation_rpy"];

        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = pos[0];
        start_pose.position.y = pos[1];
        start_pose.position.z = pos[2];

        tf2::Quaternion quat;
        quat.setRPY(rpy[0], rpy[1], rpy[2]);
        start_pose.orientation = tf2::toMsg(quat);

        std::vector<geometry_msgs::msg::Pose> path_waypoints;
        std::string shape_type = shape.contains("type") ? static_cast<std::string>(shape["type"]) : "polygon";

        if (shape_type == "arc") {
            auto center = shape["center"];
            double cx = center[0];
            double cy = center[1];
            double radius = shape["radius"];
            double start_angle = shape["start_angle"];
            double end_angle = shape["end_angle"];
            int segments = shape["segments"];

            for (int i = 0; i <= segments; ++i) {
                double theta = start_angle + (end_angle - start_angle) * i / segments;
                double x = cx + radius * cos(theta);
                double y = cy + radius * sin(theta);
                path_waypoints.push_back(transform_2d_to_3d_pose({x, y}, start_pose));
            }
        }
        else if (shape.contains("vertices")) {
            for (const auto& pt : shape["vertices"]) {
                std::pair<double, double> pt_2d = { pt[0], pt[1] };
                geometry_msgs::msg::Pose wp = transform_2d_to_3d_pose(pt_2d, start_pose);

                if (!path_waypoints.empty()) {
                    auto prev_wp = path_waypoints.back();
                    int interp_steps = 3;
                    for (int i = 1; i <= interp_steps; ++i) {
                        geometry_msgs::msg::Pose interp_pose;
                        double alpha = static_cast<double>(i) / (interp_steps + 1);
                        interp_pose.position.x = (1 - alpha) * prev_wp.position.x + alpha * wp.position.x;
                        interp_pose.position.y = (1 - alpha) * prev_wp.position.y + alpha * wp.position.y;
                        interp_pose.position.z = (1 - alpha) * prev_wp.position.z + alpha * wp.position.z;
                        interp_pose.orientation = prev_wp.orientation;
                        path_waypoints.push_back(interp_pose);
                    }
                }
                path_waypoints.push_back(wp);
            }
        }
        else if (shape_type == "bspline") {
            auto ctrl_points = shape["control_points"];
            int degree = shape.value("degree", 3);
            int num_samples = shape.value("num_points", 50);

            std::vector<Eigen::Vector2d> ctrl_vec;
            for (const auto& pt : ctrl_points) {
                ctrl_vec.emplace_back(pt[0], pt[1]);
            }

            auto spline_pts = generate_bspline_curve(ctrl_vec, degree, num_samples);
            for (const auto& pt : spline_pts) {
                path_waypoints.push_back(transform_2d_to_3d_pose({pt.x(), pt.y()}, start_pose));
            }
        }
        else {
            RCLCPP_WARN(node->get_logger(), "Unknown shape type, skipping...");
            continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node->get_clock()->now();
        marker.ns = "shape_paths";
        marker.id = marker_idx++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        for (const auto& p : path_waypoints) {
            geometry_msgs::msg::Point point;
            point.x = p.position.x;
            point.y = p.position.y;
            point.z = p.position.z;
            marker.points.push_back(point);
        }

        rclcpp::sleep_for(1s);
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub->publish(marker_array);

        move_group.setPoseTarget(start_pose);
        moveit::planning_interface::MoveGroupInterface::Plan start_plan;
        bool plan_ok = (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_ok) {
            move_group.execute(start_plan);
            rclcpp::sleep_for(1s);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to reach start pose, skipping shape.");
            continue;
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        double completion = move_group.computeCartesianPath(path_waypoints, 0.01, 0.0, trajectory);

        if (completion > 0.9) {
            RCLCPP_INFO(node->get_logger(), "Cartesian path success: %.2f%%", completion * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
            exec_plan.trajectory_ = trajectory;
            move_group.execute(exec_plan);
        } else {
            RCLCPP_WARN(node->get_logger(), "Path planning incomplete: %.2f%%, skipping.", completion * 100.0);
        }

        move_group.clearPoseTargets();
        rclcpp::sleep_for(1s);
    }

    RCLCPP_INFO(node->get_logger(), "All shape drawing tasks completed.");
    rclcpp::sleep_for(5s);
    rclcpp::shutdown();
    return 0;
}
