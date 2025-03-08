#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#define PI 3.14159265358979

void moveToTarget(moveit::planning_interface::MoveGroupInterface& move_group_arm, const std::string& target) {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //move_group_arm.setWorkspace( -0.6, -0.3, -0.2, 0.6, 1.3, 1.3);
    move_group_arm.setMaxVelocityScalingFactor(0.5);
    move_group_arm.setMaxAccelerationScalingFactor(0.5);
    move_group_arm.setPlanningTime(20);
    move_group_arm.setNamedTarget(target);
    bool success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan);
    move_group_arm.setStartStateToCurrentState();

}

int main(int argc, char *argv[])
{
    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("ur_pick_and_place_moveit");

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node =
        rclcpp::Node::make_shared("ur_pick_and_place_moveit", node_options);
    
    rclcpp::sleep_for(std::chrono::nanoseconds(100000000));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_arm(
        move_group_node, PLANNING_GROUP_ARM);

    const moveit::core::JointModelGroup *joint_model_group_arm =
        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    move_group_arm.setEndEffectorLink("tool0");

    for (auto name : move_group_arm.getLinkNames())
    {
        RCLCPP_INFO(LOGGER, "Link: %s", name.c_str());
    }
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Get Current State

    moveit::core::RobotStatePtr current_state_arm =
        move_group_arm.getCurrentState(10);

    std::vector<double> joint_group_positions_arm;
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                               joint_group_positions_arm);

    move_group_arm.setStartStateToCurrentState();
    
    RCLCPP_INFO(LOGGER, "Going point1------------------------------------");

    joint_group_positions_arm[0] = 0.00;      // Base
    joint_group_positions_arm[1] = -0.5 * PI; // Shoulder
    joint_group_positions_arm[2] = 0.00;      // Elbow
    joint_group_positions_arm[3] = -0.5 * PI; // Wrist 1
    joint_group_positions_arm[4] = 0.00;      // Wrist 2
    joint_group_positions_arm[5] = 0.00;      // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    moveToTarget(move_group_arm, "set_home");

    rclcpp::sleep_for(std::chrono::seconds(1));
//건들이지 마세요*********************************************************************

//작성해야 하는 부분 시작*******************************************************************************************
    
//입력 부분 - 라디안 값 입력 / 라디안 변환식으로 입력
 RCLCPP_INFO(LOGGER, "Going point1------------------------------------");

    //입력 부분 - 라디안 값 입력 / 라디안 변환식으로 입력
    joint_group_positions_arm[0] =  2.73; // Base
    joint_group_positions_arm[1] = -2.76; // Shoulder
    joint_group_positions_arm[2] = -1.05; // Elbow
    joint_group_positions_arm[3] = 0.67; // Wrist 1
    joint_group_positions_arm[4] = -2.73; // Wrist 2
    joint_group_positions_arm[5] = 0.00;  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_1");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
   
    
//작성해야 하는 부분 끝****************************************************************************************    
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
