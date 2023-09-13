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

    const auto& group_name = "door_opening_mechanism";   // mobile_base_arm or door_opening_mechanism
    const auto& group_name_arm = "door_opening_mechanism";
    const auto& end_effector_name = "door_opening_end_effector";
    const auto& end_effector_parent_link = "door_opening_mechanism_link_freely_rotating_hook";

    const auto& hand_group_name = "hand";
    const auto& hand_frame = "hook_base_link";

    // Set task properties
    task.setProperty("group", group_name);
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

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("Starting position", interpolation_planner);
    stage_open_hand->setGroup(group_name_arm);
    stage_open_hand->setGoal("starting_position_arm");
    stage_open_hand->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    // stage_open_hand->setIKFrame(end_effector_parent_link);
    // stage_open_hand->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_open_hand));

    // auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    //     "move to pick", mtc::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    // // clang-format on
    // stage_move_to_pick->setTimeout(5.0);
    // stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    // task.add(std::move(stage_move_to_pick));

    // auto stage_move_to_target_pose = std::make_unique<mtc::stages::MoveTo>("Move to target pose", sampling_planner);
    // stage_move_to_target_pose->setGroup(group_name);
    // stage_move_to_target_pose->setIKFrame(target_pose);
    // stage_move_to_target_pose->setGoal(target_pose);
    // task.add(std::move(stage_move_to_target_pose));

    // auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    // task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    // // clang-format off
    // grasp->properties().configureInitFrom(mtc::Stage::PARENT,
    //                                       { "eef", "group", "ik_frame" });

    /****************************************************
     *                  Generate Grasp Pose              *
     ****************************************************/
    {
      // Sample grasp pose
      // auto stage = std::make_unique<mtc::stages::GeneratePose>("Generate handle grasp pose");
      // stage->properties().configureInitFrom(mtc::Stage::PARENT);
      // stage->properties().set("marker_ns", "grasp_pose");
      // stage->setPose(target_pose);
      // stage->setMonitoredStage(current_state_ptr);   // Hook into current state

      // auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // wrapper->setMaxIKSolutions(8);
      // wrapper->setMinSolutionDistance(0.1);
      // wrapper->setIKFrame(target_pose);
      // wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      // wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      // grasp->insert(std::move(wrapper));
      // task.add(std::move(stage));
    }

    return task;
  }

}   // namespace door_opening_mechanism_mtc
