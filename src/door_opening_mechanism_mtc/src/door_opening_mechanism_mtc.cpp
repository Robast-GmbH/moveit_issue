#include "door_opening_mechanism_mtc/door_opening_mechanism_mtc.hpp"

namespace door_opening_mechanism_mtc
{

  DoorMechanismMtc::DoorMechanismMtc() : Node("door_mechanism_mtc")
  {
    handle_node_parameter();

     // Very important: We spin up the moveit interaction in new thread, otherwise
    // the current state monitor won't get any information about the robot's state.
    std::thread{std::bind(&DoorMechanismMtc::do_task, this)}
        .detach();
  }

  void DoorMechanismMtc::handle_node_parameter()
  {
    declare_parameter("moveit2_planning_group_name", _DEFAULT_PLANNING_GROUP_NAME);

    _planning_group_name = get_parameter("moveit2_planning_group_name").as_string();
  }

  void DoorMechanismMtc::do_task()
  {
    _task = create_task();

    RCLCPP_INFO(_LOGGER, "Initializing task!");
    try
    {
      _task.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(_LOGGER, e);
      return;
    }

    RCLCPP_INFO(_LOGGER, "Planning task!");
    if (!_task.plan(5 /* max_solutions */))
    {
      RCLCPP_ERROR(_LOGGER, "Task planning failed");
      return;
    }
    RCLCPP_INFO(_LOGGER, "Publishing task solutions!");
    _task.introspection().publishSolution(*_task.solutions().front());

    RCLCPP_INFO(_LOGGER, "Executing task!");
    auto result = _task.execute(*_task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(_LOGGER, "Task execution failed");
      return;
    }

    return;
  }

  mtc::Task DoorMechanismMtc::create_task()
  {
    RCLCPP_INFO(_LOGGER, "Creating task!");

    mtc::Task task;
    task.stages()->setName("approach door handle task");
    task.loadRobotModel(shared_from_this());

    const auto& group_name_arm = "door_opening_mechanism";
    const auto& end_effector_name = "door_opening_end_effector";
    const auto& end_effector_parent_link = "door_opening_mechanism_link_freely_rotating_hook";

    const auto& hand_group_name = "hand";
    const auto& hand_frame = "hook_base_link";

    // Set task properties
    task.setProperty("group", group_name_arm);
    task.setProperty("eef", end_effector_name);
    task.setProperty("ik_frame", end_effector_parent_link);

    mtc::Stage* current_state_ptr = nullptr;   // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    geometry_msgs::msg::PoseStamped target_pose = geometry_msgs::msg::PoseStamped();
    target_pose.pose.position.x = -0.680502;
    target_pose.pose.position.y = 0.211497;
    target_pose.pose.position.z = 0.822400;
    target_pose.header.frame_id = "base_footprint";

    double roll = 1.57079632679;
    double pitch = 1.57079632679;
    double yaw = 0.0;
    // Create a Quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    target_pose.pose.orientation.x = quaternion.x();
    target_pose.pose.orientation.y = quaternion.y();
    target_pose.pose.orientation.z = quaternion.z();
    target_pose.pose.orientation.w = quaternion.w();

    // This works
    auto state_move_to_start_pose = std::make_unique<mtc::stages::MoveTo>("Starting position", interpolation_planner);
    state_move_to_start_pose->setGroup(group_name_arm);
    state_move_to_start_pose->setGoal("starting_position_arm");
    state_move_to_start_pose->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    task.add(std::move(state_move_to_start_pose));

    //TODO: This does not work - why? In my opinion the target pose is reachable (compare with rviz)
    auto state_move_to_target_pose = std::make_unique<mtc::stages::MoveTo>("Move to target pose", sampling_planner);
    state_move_to_target_pose->setGroup(group_name_arm);
    state_move_to_target_pose->setGoal(target_pose);
    state_move_to_target_pose->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    state_move_to_target_pose->setIKFrame(end_effector_parent_link);
    task.add(std::move(state_move_to_target_pose));

    return task;
  }

}   // namespace door_opening_mechanism_mtc
