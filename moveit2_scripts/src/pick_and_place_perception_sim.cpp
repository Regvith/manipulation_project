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
class PerceptionSurfaceNode : public rclcpp::Node {
public:
  PerceptionSurfaceNode() : Node("perception_surface_node") {
    // Callback group for surface detection
    surface_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions surface_options;
    surface_options.callback_group = surface_callback_group_;

    surface_subscription_ =
        this->create_subscription<custom_msgs::msg::DetectedSurfaces>(
            "surface_detected", 10,
            [this](const custom_msgs::msg::DetectedSurfaces::SharedPtr msg) {
              surface_x_ = msg->position.x;
              surface_y_ = msg->position.y;
              surface_z_ = msg->position.z;
              surface_height_ = msg->height;
              surface_width_ = msg->width;
              surface_received_ = true;
              RCLCPP_INFO(this->get_logger(),
                          "Surface detected at z = %.3f, height=%.3f",
                          surface_z_, surface_height_);
            },
            surface_options);

    // Callback group for object detection
    object_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions object_options;
    object_options.callback_group = object_callback_group_;

    object_subscription_ =
        this->create_subscription<custom_msgs::msg::DetectedObjects>(
            "object_detected", 10,
            [this](const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
              object_x_ = msg->position.x;
              object_y_ = msg->position.y;
              object_z_ = msg->position.z;
              object_height_ = msg->height;
              object_width_ = msg->width;
              object_thickness_ = msg->thickness;
              object_received_ = true;
              RCLCPP_INFO(this->get_logger(),
                          "Object detected at x=%.3f y=%.3f z=%.3f height=%.3f",
                          object_x_, object_y_, object_z_, object_height_);
            },
            object_options);
  }

  // Accessors for surface
  float get_surface_x() const { return surface_x_; }
  float get_surface_y() const { return surface_y_; }
  float get_surface_z() const { return surface_z_; }
  float get_surface_height() const { return surface_height_; }
  float get_surface_width() const { return surface_width_; }
  bool surface_received() const { return surface_received_; }

  // Accessors for object
  float get_object_x() const { return object_x_; }
  float get_object_y() const { return object_y_; }
  float get_object_z() const { return object_z_; }
  float get_object_height() const { return object_height_; }
  float get_object_width() const { return object_width_; }
  float get_object_thickness() const { return object_thickness_; }
  bool object_received() const { return object_received_; }

private:
  // Subscriptions
  rclcpp::Subscription<custom_msgs::msg::DetectedSurfaces>::SharedPtr
      surface_subscription_;
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      object_subscription_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr surface_callback_group_;
  rclcpp::CallbackGroup::SharedPtr object_callback_group_;

  // Surface data
  float surface_x_ = 0.0, surface_y_ = 0.0, surface_z_ = 0.0;
  float surface_height_ = 0.0, surface_width_ = 0.0;
  bool surface_received_ = false;

  // Object data
  float object_x_ = 0.0, object_y_ = 0.0, object_z_ = 0.0;
  float object_height_ = 0.0, object_width_ = 0.0, object_thickness_ = 0.0;
  bool object_received_ = false;
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
    primitive.dimensions = {0.8, 0.02, 1.3};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.48;
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
    box_pose1.position.y = 0.0;
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
    box_pose2.position.z =
        target_z_ - s_thickness_ / 5.0 - object_height_ / 2.0;
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

    // setup_goal_pose_target(target_x_, target_y_, target_z_, q.x(), q.y(),
    // q.z(),
    //                        q.w());
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0.0, 0.0); // Z-down grasp orientation (180° around X)

    geometry_msgs::msg::Pose approach_pose;
    approach_pose.position.x = target_x_;
    approach_pose.position.y = target_y_;
    approach_pose.position.z = target_z_;
    approach_pose.orientation.x = q.x();
    approach_pose.orientation.y = q.y();
    approach_pose.orientation.z = q.z();
    approach_pose.orientation.w = q.w();

    log_pose(approach_pose, "Grasp Target Pose");

    setup_goal_pose_target(target_x_+0.012, target_y_-0.01, target_z_, q.x(), q.y(), q.z(),
                           q.w());
    // setup_goal_pose_target(target_x_, target_y_, target_z_, 0, 0, 0, 1);
    // if (!setup_ik_joint_target_from_pose(target_x_, target_y_, target_z_,
    //                                      q.x(), q.y(), q.z(), q.w())) {
    //   RCLCPP_ERROR(LOGGER, "Aborting trajectory due to failed IK.");
    //   return;
    // }
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // setup_waypoints_target(0.0, target_y_, target_z_);
    // plan_trajectory_cartesian();
    // execute_trajectory_cartesian();

    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    setup_waypoints_target(0.00, 0.00, -0.12);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    // setup_waypoints_target(0.00, 0.0, -0.1);
    // plan_trajectory_cartesian();
    // execute_trajectory_cartesian();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    closeGripper();
    
    // std::vector<double> grip_positions = {0.5,   0.55,   0.59, 0.62,
    //                                       0.625, 0.6305, 0.635};
    // for (double pos : grip_positions) {
    //   setup_joint_value_gripper(pos);
    //   plan_trajectory_gripper();
    //   execute_trajectory_gripper();
    //   std::this_thread::sleep_for(std::chrono::milliseconds(800));
    // }

    // setup_joint_value_target(0.00, -1.001, 0.00, 0.00, 0.00, 0.00);
    setup_waypoints_target(0.00, 0.00, +0.1);

    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    setup_goal_pose_target(-target_x_, target_y_, target_z_, q.x(), q.y(),
                           q.z(), q.w());
    // setup_joint_value_target(2.8406, -0.0510, -1.3290, -0.1663, -1.5785,
    //                          3.45388);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
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

  void log_pose(const geometry_msgs::msg::Pose &pose,
                const std::string &label = "Pose") {
    RCLCPP_INFO(LOGGER,
                "%s - Position: [x=%.3f, y=%.3f, z=%.3f], Orientation: "
                "[x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                label.c_str(), pose.position.x, pose.position.y,
                pose.position.z, pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w);
  }
  bool executeGripperPlan() {
    plan_trajectory_gripper();
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper closed.");
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "Gripper plan failed.");
      return false;
    }
  }
  void closeGripper() {
    float gripper_value = 0.6;
    while (gripper_value <= 0.645) {
      joint_group_positions_gripper_[2] = gripper_value;
      move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
      RCLCPP_INFO(LOGGER, "Closing gripper: %.3f.", gripper_value);
      if (this->executeGripperPlan()) {
        if (gripper_value < 0.620)
          gripper_value += 0.006;
        else if (gripper_value < 0.640)
          gripper_value += 0.004;
        else
          gripper_value += 0.001;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      } else {
        RCLCPP_ERROR(LOGGER, "Failed to close gripper. Aborting.");
        rclcpp::shutdown();
      }
    }
  }

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

    bool found_ik = kinematic_state->setFromIK(joint_model_group_robot_,
                                               target_pose, timeout);

    if (found_ik) {
      std::vector<double> joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group_robot_,
                                               joint_values);
      move_group_robot_->setJointValueTarget(joint_values);
      //   move_group_robot_->setStartStateToCurrentState();
      //   move_group_robot_->setPlanningTime(15.0);            // Default
      //   is 1.0 move_group_robot_->setNumPlanningAttempts(10);       // Try
      //   multiple plans move_group_robot_->setMaxVelocityScalingFactor(0.3);
      //   // Slower = easier
      //   move_group_robot_->setMaxAccelerationScalingFactor(0.3);

      RCLCPP_INFO(LOGGER, "IK solution found and joint target set.");
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
    cartesian_waypoints_.clear();

    Pose start = target_pose_robot_;
    cartesian_waypoints_.push_back(start);
    Pose end = start;
    end.position.x += dx;
    end.position.y += dy;
    end.position.z += dz;
    cartesian_waypoints_.push_back(end);

    RCLCPP_INFO(LOGGER, "Cartesian path: Z from %.3f to %.3f", start.position.z,
                end.position.z);
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
  auto detection_node = std::make_shared<PerceptionSurfaceNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(detection_node);

  RCLCPP_INFO(LOGGER, "Waiting for object and surface detection...");
  auto start_time = std::chrono::steady_clock::now();
  while ((!detection_node->object_received() ||
          !detection_node->surface_received())) {
    exec.spin_some();
  }

  if (!detection_node->object_received() ||
      !detection_node->surface_received()) {
    RCLCPP_ERROR(LOGGER, "Detection timeout.");
    rclcpp::shutdown();
    return 1;
  }

  float x = detection_node->get_object_x();
  float y = detection_node->get_object_y();
  float object_z = detection_node->get_object_z();
  float height = detection_node->get_surface_height();
  float object_height = detection_node->get_object_height();
  float surface_z = detection_node->get_surface_z();
  auto round3 = [](float value) {
    return std::round(value * 1000.0f) / 1000.0f; // Round to 3 decimal places
  };

  object_z = round3(detection_node->get_object_z());
  surface_z = round3(detection_node->get_surface_z());
  height = round3(detection_node->get_surface_height());
  object_height = round3(detection_node->get_object_height());

  float grasp_z = object_z + height / 5.0 + object_height / 2.0;
  grasp_z = round3(grasp_z);
  RCLCPP_INFO(LOGGER, "Grasp target: x=%.2f y=%.2f z=%.2f", x, y, grasp_z);

  PickAndPlaceTrajectory pick_and_place(
      base_node, x, y, grasp_z, detection_node->get_object_height(),
      detection_node->get_object_width(),
      detection_node->get_object_thickness(), detection_node->get_surface_x(),
      detection_node->get_surface_y(), detection_node->get_surface_z(),
      detection_node->get_surface_width(),
      detection_node->get_surface_height());

  pick_and_place.execute_trajectory_plan();

  rclcpp::shutdown();
  return 0;
}
