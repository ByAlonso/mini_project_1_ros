#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


cubes_position = {}
bucket_position = {}
num_cubes = 0;

def get_element_position(element_position):
  return {"position_x": element_position.position.x,"position_y": element_position.position.y,"position_z": element_position.position.z}

def pose_cubes(Pose_cube_msg):
  num_cubes = len(Pose_cube_msg.name) - 3
  for i in range(0,num_cubes):
    cube = 'cube' + str(i)
    cubes_position[cube] = get_element_position(Pose_cube_msg.pose[(Pose_cube_msg.name).index(cube)])
  bucket_position['bucket'] = get_element_position(Pose_cube_msg.pose[(Pose_cube_msg.name).index('bucket')])

def get_model_elements():
  msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
  pose_cubes(msg)

def plan_and_move(key, robot, group, pose_goal, position_x, position_y, position_z, display_trajectory_publisher, opened , closed):
  rospy.sleep(0.5)
  ## Planning to a Pose goal
  print ("============ Generating plan " + key)
  pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_goal.position.x = position_x
  pose_goal.position.y = position_y
  pose_goal.position.z = position_z
  print pose_goal
  group.set_pose_target(pose_goal)
  plan = group.plan()
  if not plan.joint_trajectory.points:
    print("No posible movement found")
    print("returning to initial position")
    return False
  print ("============ Visualizing " + key)
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  display_trajectory_publisher.publish(display_trajectory);
  rospy.sleep(2.)
  ## Moving to a pose goal
  group.go(wait=True)

  if(opened):
    execfile('/home/alonso/catkin_ws/src/hello_ros/scripts/proyect/project_open_gripper.py')
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x = position_x
    pose_goal.position.y = position_y
    pose_goal.position.z = 1.4
    group.set_pose_target(pose_goal)
    plan = group.plan()
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);
    rospy.sleep(2.)
    ## Moving to a pose goal
    group.go(wait=True)
  elif(closed):
    execfile('/home/alonso/catkin_ws/src/hello_ros/scripts/proyect/project_close_gripper.py')
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x = position_x
    pose_goal.position.y = position_y
    pose_goal.position.z = 1.3
    group.set_pose_target(pose_goal)
    plan = group.plan()
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);
    rospy.sleep(2.)
    ## Moving to a pose goal
    group.go(wait=True)
    
  return True

def move_group_python_interface_tutorial():
  # BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
 
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
 
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
  print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  get_model_elements()
  
  ## Let's setup the planner
  #group.set_planning_time(30)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)

  #If we're coming from another script we might want to remove the objects
  if "table" in scene.get_known_object_names():
    scene.remove_world_object("table")
  if "table2" in scene.get_known_object_names():
    scene.remove_world_object("table2")
  if "groundplane" in scene.get_known_object_names():
    scene.remove_world_object("groundplane")

  init_pose = group.get_current_pose().pose
  pose_goal = group.get_current_pose().pose
  plan = True
  for key in cubes_position:
    if plan:
      plan = plan_and_move(key,robot,group,pose_goal,cubes_position[key]['position_x'],cubes_position[key]['position_y'], cubes_position[key]['position_z'] + 0.17, display_trajectory_publisher, False, True)
    if plan:
      plan = plan_and_move(key,robot,group,pose_goal,bucket_position['bucket']['position_x'],bucket_position['bucket']['position_y'], 1.3, display_trajectory_publisher, True, False)
    if not plan:
      plan = plan_and_move(key,robot,group,pose_goal,init_pose.position.x,init_pose.position.y, init_pose.position.z, display_trajectory_publisher, False, False)
    plan = True
    rospy.sleep(0.5)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()
if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass