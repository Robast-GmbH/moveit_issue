#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

namespace mtc = moveit::task_constructor;

namespace door_opening_mechanism_mtc
{
  class DoorMechanismMtc : public rclcpp::Node
  {
   public:
    DoorMechanismMtc();

   private:
    const rclcpp::Logger _LOGGER = rclcpp::get_logger("dom_mtc");


    std::string _planning_group_name;
    const std::string _DEFAULT_PLANNING_GROUP_NAME = "door_opening_mechanism";

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr _door_handle_position_subscription;
    mtc::Task _task;

    void handle_node_parameter();

    mtc::Task create_task();

    void do_task();

  };
}   // namespace door_opening_mechanism_mtc
