#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("xarm_motionplanning_node", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // MoveIt move_group interface
  static const std::string PLANNING_GROUP = "xarm6";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


//   // Planning with Path Constraints
//   move_group.setPoseReferenceFrame("base_link");
//   move_group.setEndEffectorLink("J5_wrist2/body");

//   moveit_msgs::msg::OrientationConstraint ocm;
//   ocm.link_name = "J5_wrist2/body";
//   ocm.header.frame_id = "base_link";
//   auto cur = move_group.getCurrentPose("J5_wrist2/body");
//   ocm.orientation = cur.pose.orientation;
// //   ocm.orientation.w = 1.0;
// //   ocm.orientation.x = 0.0;
// //   ocm.orientation.y = 0.0;
// //   ocm.orientation.z = 0.0;
//   ocm.absolute_x_axis_tolerance = 1.0;
//   ocm.absolute_y_axis_tolerance = 1.0;
//   ocm.absolute_z_axis_tolerance = 1.0;
//   ocm.weight = 1.0;

//   moveit_msgs::msg::Constraints path_constraints;
//   path_constraints.orientation_constraints.push_back(ocm);
//   move_group.setPathConstraints(path_constraints);




//   // Goal Constraints
//   auto add_joint_range = [&](const std::string& joint, double center, double tol_above, double tol_below, double weight=1.0)
//   {
//     moveit_msgs::msg::Constraints target_constraint;
//     moveit_msgs::msg::JointConstraint joint_constraint;

//     joint_constraint.joint_name = joint;
//     joint_constraint.position = center;
//     joint_constraint.tolerance_above = tol_above;
//     joint_constraint.tolerance_below = tol_below;
//     joint_constraint.weight = weight;

//     target_constraint.joint_constraints.push_back(joint_constraint);

//     move_group.setPathConstraints(target_constraint);
//   };

//   move_group.clearPathConstraints();
//   add_joint_range("J1_base", 0.0, M_PI/2, -M_PI/2);
//   add_joint_range("J2_shoulder", 0.0, M_PI, -M_PI/3);
//   add_joint_range("J3_elbow", 0.0, M_PI*2/3, 0.0);
//   add_joint_range("J4_wrist1", 0.0, M_PI, -M_PI/2);
//   add_joint_range("J5_wrist2", 0.0, M_PI/2, -M_PI/2);




  // Planning to a Pose goal
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x =  0.4;
  target_pose.position.y =  0.0;
  target_pose.position.z =  0.4;
  target_pose.orientation.w = 0.0;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  move_group.setPoseTarget(target_pose);



  // // Planning to a joint-space
  // std::vector<double> joint_group_positions(5);
  // joint_group_positions[0] =  0.0;  // radians
  // joint_group_positions[1] =  0.0;
  // joint_group_positions[2] =  0.0;
  // joint_group_positions[3] =  0.0;
  // joint_group_positions[4] =  0.0;

  // move_group.setJointValueTarget(joint_group_positions);
  // if (!within_bounds)
  // {
  //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  // }




  // Plan to the original target pose
  // move_group.setPlanningTime(10.0);
  // move_group.setNumPlanningAttempts(10);

  move_group.move();


  // for(int i=0; i<5; i++){
  //   target_pose.position.x =  0.4;
  //   target_pose.position.y =  0.0;
  //   target_pose.position.z =  0.4;
  //   target_pose.orientation.w = 0.0;
  //   target_pose.orientation.x = 1.0;
  //   target_pose.orientation.y = 0.0;
  //   target_pose.orientation.z = 0.0;
  //   move_group.setPoseTarget(target_pose);

  //   move_group.move();

    
  //   target_pose.position.x =  0.4;
  //   target_pose.position.y =  0.0;
  //   target_pose.position.z =  0.4;
  //   target_pose.orientation.w = 0.3827;
  //   target_pose.orientation.x = 0.9239;
  //   target_pose.orientation.y = 0.0;
  //   target_pose.orientation.z = 0.0;
  //   move_group.setPoseTarget(target_pose);

  //   move_group.move();
  // }



//   // Enforce Planning in Joint Space
//   moveit::core::RobotState start_state(*move_group.getCurrentState());
//   geometry_msgs::msg::Pose start_pose2;
//   start_pose2.orientation.w = 1.0;
//   start_pose2.position.x = 0.55;
//   start_pose2.position.y = -0.05;
//   start_pose2.position.z = 0.8;
//   start_state.setFromIK(joint_model_group, start_pose2);
//   move_group.setStartState(start_state);

//   // Now, we will plan to the earlier pose target from the new
//   // start state that we just created.
//   move_group.setPoseTarget(target_pose);

//   // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
//   // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
//   move_group.setPlanningTime(10.0);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
// //   RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

//   // When done with the path constraint, be sure to clear it.
//   move_group.clearPathConstraints();



//   // Cartesian Paths
//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   waypoints.push_back(start_pose2);

//   geometry_msgs::msg::Pose target_pose3 = start_pose2;

//   target_pose3.position.z -= 0.2;
//   waypoints.push_back(target_pose3);  // down

//   target_pose3.position.y -= 0.2;
//   waypoints.push_back(target_pose3);  // right

//   target_pose3.position.z += 0.2;
//   target_pose3.position.y += 0.2;
//   target_pose3.position.x -= 0.2;
//   waypoints.push_back(target_pose3);  // up and left

//   // We want the Cartesian path to be interpolated at a resolution of 1 cm,
//   // which is why we will specify 0.01 as the max step in Cartesian translation.
//   const double eef_step = 0.01;
//   moveit_msgs::msg::RobotTrajectory trajectory;
//   double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of Cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.
  //
  // You can execute a trajectory like this.
  /* move_group.execute(trajectory); */

//   // Adding objects to the environment
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // First, let's plan to another simple goal with no objects in the way.
//   move_group.setStartState(*move_group.getCurrentState());
//   geometry_msgs::msg::Pose another_pose;
//   another_pose.orientation.w = 0;
//   another_pose.orientation.x = -1.0;
//   another_pose.position.x = 0.7;
//   another_pose.position.y = 0.0;
//   another_pose.position.z = 0.59;
//   move_group.setPoseTarget(another_pose);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishAxisLabeled(another_pose, "goal");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_clear_path.gif
//   //    :alt: animation showing the arm moving relatively straight toward the goal
//   //
//   // Now, let's define a collision object ROS message for the robot to avoid.
//   moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object.id = "box1";

//   // Define a box to add to the world.
//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 0.1;
//   primitive.dimensions[primitive.BOX_Y] = 1.5;
//   primitive.dimensions[primitive.BOX_Z] = 0.5;

//   // Define a pose for the box (specified relative to frame_id).
//   geometry_msgs::msg::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.48;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 0.25;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   RCLCPP_INFO(LOGGER, "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//   visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//   // Now, when we plan a trajectory it will avoid the obstacle.
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//   visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//   //    :alt: animation showing the arm moving avoiding the new obstacle
//   //
//   // Attaching objects to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // You can attach an object to the robot, so that it moves with the robot geometry.
//   // This simulates picking up the object for the purpose of manipulating it.
//   // The motion planning should avoid collisions between objects as well.
//   moveit_msgs::msg::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::msg::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper.
//   object_to_attach.header.frame_id = move_group.getEndEffectorLink();
//   geometry_msgs::msg::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.2;

//   // First, we add the object to the world (without using a vector).
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);

//   // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
//   // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
//   // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
//   RCLCPP_INFO(LOGGER, "Attach the object to the robot");
//   std::vector<std::string> touch_links;
//   touch_links.push_back("panda_rightfinger");
//   touch_links.push_back("panda_leftfinger");
//   move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

//   visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

//   // Replan, but now with the object in hand.
//   move_group.setStartStateToCurrentState();
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look something like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_attached_object.gif
//   //    :alt: animation showing the arm moving differently once the object is attached
//   //
//   // Detaching and Removing Objects
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Now, let's detach the cylinder from the robot's gripper.
//   RCLCPP_INFO(LOGGER, "Detach the object from the robot");
//   move_group.detachObject(object_to_attach.id);

//   // Show text in RViz of status
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

//   // Now, let's remove the objects from the world.
//   RCLCPP_INFO(LOGGER, "Remove the objects from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   object_ids.push_back(object_to_attach.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

//   // END_TUTORIAL
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();



  rclcpp::shutdown();
  return 0;
}