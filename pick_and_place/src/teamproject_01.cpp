#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#define PI 3.14159265358979
//건들이지 마세요**********************************************************************
void make_Collision(moveit::planning_interface::PlanningSceneInterface planning_scene_interface)
{
    // Make collosion to appropriate path planning start
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(8);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "back";
    collision_objects[0].header.frame_id = "base_link";

    //Define the primitive and its dimensions.
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 0.001;
    collision_objects[0].primitives[0].dimensions[2] = 0.75;

    //Define the pose of the table.
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = -0.3;
    collision_objects[0].primitive_poses[0].position.z = 0.375;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "right";
    collision_objects[1].header.frame_id = "base_link";

    //Define the primitive and its dimensions.
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.001;
    collision_objects[1].primitives[0].dimensions[1] = 1.0;
    collision_objects[1].primitives[0].dimensions[2] = 0.75;

    //Define the pose of the table.
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.6;
    collision_objects[1].primitive_poses[0].position.y = 0.2;
    collision_objects[1].primitive_poses[0].position.z = 0.375;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].id = "left";

    //Define the primitive and its dimensions.
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.001;
    collision_objects[2].primitives[0].dimensions[1] = 1.0;
    collision_objects[2].primitives[0].dimensions[2] = 0.75;

    //Define the pose of the object.
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = -0.6;
    collision_objects[2].primitive_poses[0].position.y = 0.2;
    collision_objects[2].primitive_poses[0].position.z = 0.375;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    collision_objects[3].header.frame_id = "base_link";
    collision_objects[3].id = "ur_floor";
       
     /* Define the primitive and its dimensions. */
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.9;
    collision_objects[3].primitives[0].dimensions[1] = 0.9;
    collision_objects[3].primitives[0].dimensions[2] = 0.9;

    /* Define the pose of the object. */
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.0;
    collision_objects[3].primitive_poses[0].position.y = 0.25;
    collision_objects[3].primitive_poses[0].position.z = -0.45;
    collision_objects[3].primitive_poses[0].orientation.w = 0.0;
    
    // END_SUB_TUTORIAL

    collision_objects[3].operation = collision_objects[3].ADD;   
    
    collision_objects[4].header.frame_id = "base_link";
    collision_objects[4].id = "water_cup";

    /* Define the primitive and its dimensions. */
    collision_objects[4].primitives.resize(1);
    collision_objects[4].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
    collision_objects[4].primitives[0].dimensions.resize(2);
    collision_objects[4].primitives[0].dimensions[0] = 0.07;
    collision_objects[4].primitives[0].dimensions[1] = 0.03;

    /* Define the pose of the object. */
    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].position.x = 0.4;
    collision_objects[4].primitive_poses[0].position.y = 0.65;
    collision_objects[4].primitive_poses[0].position.z = 0.19;
    collision_objects[4].primitive_poses[0].orientation.w = 1.0;

    collision_objects[4].operation = collision_objects[4].ADD;

    collision_objects[5].header.frame_id = "base_link";
    collision_objects[5].id = "mini_table1";

    /* Define the primitive and its dimensions. */
    collision_objects[5].primitives.resize(1);
    collision_objects[5].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[5].primitives[0].dimensions.resize(3);
    collision_objects[5].primitives[0].dimensions[0] = 0.08;
    collision_objects[5].primitives[0].dimensions[1] = 0.08;
    collision_objects[5].primitives[0].dimensions[2] = 0.15;

    /* Define the pose of the object. */
    collision_objects[5].primitive_poses.resize(1);
    collision_objects[5].primitive_poses[0].position.x = 0.4;
    collision_objects[5].primitive_poses[0].position.y = 0.65;
    collision_objects[5].primitive_poses[0].position.z = 0.075;
    collision_objects[5].primitive_poses[0].orientation.w = 1.0;

    collision_objects[5].operation = collision_objects[5].ADD;

    collision_objects[6].header.frame_id = "base_link";
    collision_objects[6].id = "empty_cup";

    /* Define the primitive and its dimensions. */
    collision_objects[6].primitives.resize(1);
    collision_objects[6].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
    collision_objects[6].primitives[0].dimensions.resize(2);
    collision_objects[6].primitives[0].dimensions[0] = 0.07;
    collision_objects[6].primitives[0].dimensions[1] = 0.03;

    /* Define the pose of the object. */
    collision_objects[6].primitive_poses.resize(1);
    collision_objects[6].primitive_poses[0].position.x = -0.35;
    collision_objects[6].primitive_poses[0].position.y = 0.6;
    collision_objects[6].primitive_poses[0].position.z = 0.07;
    collision_objects[6].primitive_poses[0].orientation.w = 1.0;

    collision_objects[6].operation = collision_objects[6].ADD;

    collision_objects[7].header.frame_id = "base_link";
    collision_objects[7].id = "mini_table2";

    /* Define the primitive and its dimensions. */
    collision_objects[7].primitives.resize(1);
    collision_objects[7].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[7].primitives[0].dimensions.resize(3);
    collision_objects[7].primitives[0].dimensions[0] = 0.08;
    collision_objects[7].primitives[0].dimensions[1] = 0.08;
    collision_objects[7].primitives[0].dimensions[2] = 0.03;

    /* Define the pose of the object. */
    collision_objects[7].primitive_poses.resize(1);
    collision_objects[7].primitive_poses[0].position.x = -0.35;
    collision_objects[7].primitive_poses[0].position.y = 0.6;
    collision_objects[7].primitive_poses[0].position.z = 0.015;
    collision_objects[7].primitive_poses[0].orientation.w = 1.0;

    collision_objects[7].operation = collision_objects[7].ADD;
    

    planning_scene_interface.applyCollisionObjects(collision_objects);
    // Make collosion to appropriate path planning finish
}
//건들이지 마세요**********************************************************************
void moveToTarget(moveit::planning_interface::MoveGroupInterface& move_group_arm, const std::string& target) {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group_arm.setWorkspace( -0.6, -0.3, -0.2, 0.6, 1.3, 1.3);
    move_group_arm.setMaxVelocityScalingFactor(0.5);
    move_group_arm.setMaxAccelerationScalingFactor(0.5);
    move_group_arm.setPlanningTime(20);
    move_group_arm.setNamedTarget(target);
    bool success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan);
    move_group_arm.setStartStateToCurrentState();

}
//건들이지 마세요**********************************************************************
int main(int argc, char *argv[])
{
//건들이지 마세요**********************************************************************
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
    make_Collision(planning_scene_interface);

    // Get Current State

    moveit::core::RobotStatePtr current_state_arm =
        move_group_arm.getCurrentState(10);

    std::vector<double> joint_group_positions_arm;
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                               joint_group_positions_arm);

    move_group_arm.setStartStateToCurrentState();
    // Set UR5 to Home State//---------------------------------------------
    RCLCPP_INFO(LOGGER, "Going Home------------------------------------");

    joint_group_positions_arm[0] = 0.00;      // Base
    joint_group_positions_arm[1] = -0.5 * PI; // Shoulder
    joint_group_positions_arm[2] = 0.00;      // Elbow
    joint_group_positions_arm[3] = -0.5 * PI; // Wrist 1
    joint_group_positions_arm[4] = 0.00;      // Wrist 2
    joint_group_positions_arm[5] = 0.00;      // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    moveToTarget(move_group_arm, "set_home");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
//건들이지 마세요*********************************************************************

//작성해야 하는 부분 시작*******************************************************************************************
    //----------------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Going point1------------------------------------");

    //입력 부분 - 라디안 값 입력 / 라디안 변환식으로 입력
    joint_group_positions_arm[0] =  (36) *(PI/180); // Base
    joint_group_positions_arm[1] = (-59) *(PI/180); // Shoulder
    joint_group_positions_arm[2] = (84) *(PI/180); // Elbow
    joint_group_positions_arm[3] = (-32) *(PI/180); // Wrist 1
    joint_group_positions_arm[4] =  (34) *(PI/180); // Wrist 2
    joint_group_positions_arm[5] = (4) *(PI/180);  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_1");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //-----------------------------------------------------------------------

//물체 잡기--------------------------------------
    move_group_arm.attachObject("water_cup", "tool0");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
//-------------------------------------------------
 
    //-----------------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Going point2------------------------------------");

    joint_group_positions_arm[0] =  (36) *(PI/180); // Base
    joint_group_positions_arm[1] = (-63) *(PI/180); // Shoulder
    joint_group_positions_arm[2] = (81) *(PI/180); // Elbow
    joint_group_positions_arm[3] = (-25) *(PI/180); // Wrist 1
    joint_group_positions_arm[4] =  (34) *(PI/180); // Wrist 2
    joint_group_positions_arm[5] = (4) *(PI/180);  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    
    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_2");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //------------------------------------------------------------------------

    //------------------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Going point3------------------------------------");

    joint_group_positions_arm[0] =  (114) *(PI/180); // Base
    joint_group_positions_arm[1] = (-68) *(PI/180); // Shoulder
    joint_group_positions_arm[2] = (101) *(PI/180); // Elbow
    joint_group_positions_arm[3] = (-37) *(PI/180); // Wrist 1
    joint_group_positions_arm[4] =  (111) *(PI/180); // Wrist 2
    joint_group_positions_arm[5] = (-4) *(PI/180);  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_3");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Going point4------------------------------------");

    joint_group_positions_arm[0] =  (114) *(PI/180); // Base
    joint_group_positions_arm[1] = (-68) *(PI/180); // Shoulder
    joint_group_positions_arm[2] = (101) *(PI/180); // Elbow
    joint_group_positions_arm[3] = (-37) *(PI/180); // Wrist 1
    joint_group_positions_arm[4] =  (111) *(PI/180); // Wrist 2
    joint_group_positions_arm[5] = (-113) *(PI/180);  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_4");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Going point5------------------------------------");

    joint_group_positions_arm[0] =  (36) *(PI/180); // Base
    joint_group_positions_arm[1] = (-63) *(PI/180); // Shoulder
    joint_group_positions_arm[2] = (81) *(PI/180); // Elbow
    joint_group_positions_arm[3] = (-25) *(PI/180); // Wrist 1
    joint_group_positions_arm[4] =  (34) *(PI/180); // Wrist 2
    joint_group_positions_arm[5] = (4) *(PI/180);  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_5");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //----------------------------------------------------------------------------

    //----------------------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Going point6------------------------------------");

    //입력 부분 - 라디안 값 입력 / 라디안 변환식으로 입력
    joint_group_positions_arm[0] =  (36) *(PI/180); // Base
    joint_group_positions_arm[1] = (-59) *(PI/180); // Shoulder
    joint_group_positions_arm[2] = (84) *(PI/180); // Elbow
    joint_group_positions_arm[3] = (-32) *(PI/180); // Wrist 1
    joint_group_positions_arm[4] =  (34) *(PI/180); // Wrist 2
    joint_group_positions_arm[5] = (4) *(PI/180);  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    //입력 부분 - my_plan_arm0에서 숫자 하나씩 올리기
    moveToTarget(move_group_arm, "my_plan_arm_6");

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //-----------------------------------------------------------------------------

//물체 놓기-----------------------------------------
    move_group_arm.detachObject("water_cup");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
//-------------------------------------------------

//작성해야 하는 부분 끝****************************************************************************************
    RCLCPP_INFO(LOGGER, "Going Back Home-------------------------------------");

    joint_group_positions_arm[0] = 0.00;      // Base
    joint_group_positions_arm[1] = -0.5 * PI; // Shoulder
    joint_group_positions_arm[2] = 0.00;      // Elbow
    joint_group_positions_arm[3] = -0.5 * PI; // Wrist 1
    joint_group_positions_arm[4] = 0.00;      // Wrist 2
    joint_group_positions_arm[5] = 0.00;      // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    moveToTarget(move_group_arm, "back_home");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
