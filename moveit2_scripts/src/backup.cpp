#include <chrono>
#include <cmath>
#include <custom_msgs/msg/detected_objects.hpp>
#include <custom_msgs/msg/detected_surfaces.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class SurfaceNode : public rclcpp::Node {
public:
  SurfaceNode() : Node("surface_node") {
    subscription_ =
        this->create_subscription<custom_msgs::msg::DetectedSurfaces>(
            "surface_detected", 10,
            [this](const custom_msgs::msg::DetectedSurfaces::SharedPtr msg) {
              surface_x_ = msg->position.x;
              surface_y_ = msg->position.y;
              surface_z_ = msg->position.z;
              height_ = msg->height;
              width_ = msg->width;

              received_ = true;
              RCLCPP_INFO(this->get_logger(), "Surface detected at z = %.3f",
                          surface_z_);
            });
  }

  float get_surface_x() const { return surface_x_; }
  float get_surface_y() const { return surface_y_; }
  float get_surface_z() const { return surface_z_; }
  float get_height() const { return height_; }
  float get_width() const { return width_; }

  bool received() const { return received_; }

private:
  rclcpp::Subscription<custom_msgs::msg::DetectedSurfaces>::SharedPtr
      subscription_;
  float surface_x_ = 0.0, surface_y_ = 0.0, surface_z_ = 0.0, width_ = 0.0,
        height_ = 0.0;
  bool received_ = false;
};
class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {
    subscription_ =
        this->create_subscription<custom_msgs::msg::DetectedObjects>(
            "object_detected", 10,
            [this](const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
              x_ = msg->position.x;
              y_ = msg->position.y;
              z_ = msg->position.z;
              height_ = msg->height;
              width_ = msg->width;
              thickness_ = msg->thickness; // Fixed from msg->get_thickness
              received_ = true;
              RCLCPP_INFO(this->get_logger(),
                          "Received object at x=%.3f y=%.3f z=%.3f height=%.3f",
                          x_, y_, z_, height_);
            });
  }

  float get_x() const { return x_; }
  float get_y() const { return y_; }
  float get_z() const { return z_; }
  float get_height() const { return height_; }
  float get_width() const { return width_; }
  float get_thickness() const { return thickness_; } // Fixed semicolon

  bool received() const { return received_; }

private:
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      subscription_;
  float x_ = 0.0, y_ = 0.0, z_ = 0.0, height_ = 0.0, width_ = 0.0,
        thickness_ = 0.0; // Fixed var name
  bool received_ = false;
};

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node, float x, float y,
                         float z, float object_height, float object_width,
                         float object_thickness, float s_x, float s_y,
                         float s_z, float s_width, float s_thickness)
      : base_node_(base_node), target_x_(x), target_y_(y), target_z_(z),
        object_height_(object_height), object_thickness_(object_thickness),
        object_width_(object_width), s_x_(s_x), s_y_(s_y), s_z_(s_z),
        s_height_(s_width), s_thickness_(s_thickness) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    // Add collision object to planning scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.8, 0.1, 0.8};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.52;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObjects({collision_object});
    moveit_msgs::msg::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group_robot_->getPlanningFrame();
    collision_object1.id = "box2";

    shape_msgs::msg::SolidPrimitive primitive1;
    primitive1.type = primitive.BOX;
    primitive1.dimensions = {0.5, 0.8, 0.02};

    geometry_msgs::msg::Pose box_pose1;
    box_pose1.orientation.w = 1.0;
    box_pose1.position.x = s_x_;
    box_pose1.position.y = s_y_ + 0.2;
    box_pose1.position.z = s_z_;

    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(box_pose1);
    planning_scene_interface.applyCollisionObjects({collision_object1});
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group_robot_->getPlanningFrame();
    collision_object2.id = "box3";

    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions = {object_width_, object_thickness, object_height};
    tf2::Quaternion q;
    q.setRPY(1.57, 0.0, 0.0);
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.position.x = target_x_;
    box_pose2.position.y = target_y_;
    box_pose2.position.z = target_z_;
    box_pose2.orientation.x = q.x();
    box_pose2.orientation.y = q.y();
    box_pose2.orientation.z = q.z();
    box_pose2.orientation.w = q.w();
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    planning_scene_interface.applyCollisionObjects({collision_object2});
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Trajectory...");

    tf2::Quaternion q;
    q.setRPY(1.57, 0.0, 0.0); // 90 degrees yaw
    setup_ik_joint_target_from_pose(target_x_, target_y_, target_z_, q.x(),
                                    q.y(), q.z(), q.w());
    if (!setup_ik_joint_target_from_pose(target_x_, target_y_, target_z_, q.x(),
                                         q.y(), q.z(), q.w())) {
      RCLCPP_ERROR(LOGGER, "Aborting trajectory due to failed IK.");
      return;
    }
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // RCLCPP_INFO(LOGGER, "Opening Gripper...");
    // setup_named_pose_gripper("open");
    // plan_trajectory_gripper();
    // execute_trajectory_gripper();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // setup_waypoints_target(0.003, 0.0085, -0.08);
    // plan_trajectory_cartesian();
    // execute_trajectory_cartesian();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // std::vector<double> grip_positions = {0.5,  0.52, 0.55, 0.57,
    //                                       0.59, 0.61, 0.63};
    // for (double pos : grip_positions) {
    //   setup_joint_value_gripper(pos);
    //   plan_trajectory_gripper();
    //   execute_trajectory_gripper();
    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }

    // setup_joint_value_target(-2.5341, -1.5621, 0.00, 0.00, 0.00, 0.00);
    // plan_trajectory_kinematics();
    // execute_trajectory_kinematics();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    // setup_joint_value_target(2.8406, -0.0510, -1.3290, -0.1663, -1.5785,
    //                          -3.45388);
    // plan_trajectory_kinematics();
    // execute_trajectory_kinematics();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    // RCLCPP_INFO(LOGGER, "Opening Gripper...");
    // setup_named_pose_gripper("open");
    // plan_trajectory_gripper();
    // execute_trajectory_gripper();
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  const std::vector<std::string> &joint_names =
      joint_model_group->getVariableNames();
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;
  RobotStatePtr current_state_robot_, current_state_gripper_;
  std::vector<double> joint_group_positions_robot_;
  std::vector<double> joint_group_positions_gripper_;
  Plan kinematics_trajectory_plan_, gripper_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false, plan_success_gripper_ = false;
  std::vector<Pose> cartesian_waypoints_;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;
  float target_x_, target_y_, target_z_, object_height_, object_thickness_,
      object_width_, s_x_, s_y_, s_z_, s_height_, s_thickness_;
  bool setup_ik_joint_target_from_pose(float x, float y, float z, float qx,
                                       float qy, float qz, float qw) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    moveit::core::RobotStatePtr kinematic_state =
        move_group_robot_->getCurrentState(10);
    const double timeout = 0.1;
    // Log current joint names and values
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(),
                  joint_values[i]);
    }

    // Attempt to find an IK solution
    bool found_ik = kinematic_state->setFromIK(joint_model_group_robot_,
                                               target_pose, timeout);

    if (found_ik) {
      std::vector<double> ik_joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group_robot_,
                                               ik_joint_values);

      if (ik_joint_values.size() >=
          6) // Ensure enough joint values are available
      {
        move_group_robot_->setup_joint_value_target(
            {ik_joint_values[0], ik_joint_values[1], ik_joint_values[2],
             ik_joint_values[3], ik_joint_values[4], ik_joint_values[5]});
        RCLCPP_INFO(LOGGER, "IK solution found and joint target set.");
      } else {
        RCLCPP_ERROR(LOGGER,
                     "IK returned insufficient joint values: expected at least "
                     "6, got %zu",
                     ik_joint_values.size());
        return false;
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to find IK solution.");
    }

    return found_ik;
  }

  void setup_goal_pose_target(float x, float y, float z, float qx, float qy,
                              float qz, float qw) {
    target_pose_robot_.position.x = x;
    target_pose_robot_.position.y = y;
    target_pose_robot_.position.z = z;
    target_pose_robot_.orientation.x = qx;
    target_pose_robot_.orientation.y = qy;
    target_pose_robot_.orientation.z = qz;
    target_pose_robot_.orientation.w = qw;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void setup_waypoints_target(float dx, float dy, float dz) {
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.clear();
    cartesian_waypoints_.push_back(target_pose_robot_);
    target_pose_robot_.position.x += dx;
    target_pose_robot_.position.y += dy;
    target_pose_robot_.position.z += dz;
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void setup_joint_value_target(float a0, float a1, float a2, float a3,
                                float a4, float a5) {
    joint_group_positions_robot_ = {a0, a1, a2, a3, a4, a5};
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_joint_value_gripper(float angle) {
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(const std::string &pose_name) {
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_kinematics() {
    plan_success_robot_ =
        move_group_robot_->plan(kinematics_trajectory_plan_) ==
        moveit::core::MoveItErrorCode::SUCCESS;
  }

  void execute_trajectory_kinematics() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success!");
    } else {
      RCLCPP_WARN(LOGGER, "Robot Kinematics Trajectory Failed!");
    }
  }

  void plan_trajectory_cartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    if (plan_fraction_robot_ >= 0.0) {
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success!");
    } else {
      RCLCPP_WARN(LOGGER, "Robot Cartesian Trajectory Failed!");
    }
    cartesian_waypoints_.clear();
  }

  void plan_trajectory_gripper() {
    move_group_gripper_->setMaxVelocityScalingFactor(0.05);
    move_group_gripper_->setMaxAccelerationScalingFactor(0.05);
    plan_success_gripper_ =
        move_group_gripper_->plan(gripper_trajectory_plan_) ==
        moveit::core::MoveItErrorCode::SUCCESS;
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Success!");
    } else {
      RCLCPP_WARN(LOGGER, "Gripper Action Failed!");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto base_node = std::make_shared<rclcpp::Node>("pick_and_place_trajectory");
  auto perception_node = std::make_shared<PerceptionNode>();
  auto surface_node = std::make_shared<SurfaceNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(perception_node);
  exec.add_node(surface_node);

  RCLCPP_INFO(LOGGER, "Waiting for object and surface detection...");
  auto start_time = std::chrono::steady_clock::now();
  while ((!perception_node->received() || !surface_node->received()) &&
         std::chrono::steady_clock::now() - start_time <
             std::chrono::seconds(5)) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!perception_node->received() || !surface_node->received()) {
    RCLCPP_ERROR(LOGGER, "Detection timeout.");
    rclcpp::shutdown();
    return 1;
  }

  float x = perception_node->get_x();
  float y = perception_node->get_y();
  float object_z = perception_node->get_z();
  float height = perception_node->get_height();
  float surface_z = surface_node->get_surface_z();
  float grasp_z = object_z;

  RCLCPP_INFO(LOGGER, "Grasp target: x=%.2f y=%.2f z=%.2f", x, y, grasp_z);

  PickAndPlaceTrajectory pick_and_place(
      base_node, x, y, grasp_z, perception_node->get_height(),
      perception_node->get_width(), perception_node->get_thickness(),
      surface_node->get_surface_x(), surface_node->get_surface_y(),
      surface_node->get_surface_z(), surface_node->get_width(),
      surface_node->get_height());
  pick_and_place.execute_trajectory_plan();

  rclcpp::shutdown();
  return 0;
}
