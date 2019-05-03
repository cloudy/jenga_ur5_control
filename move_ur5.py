#!/usr/bin/env python

import sys
import copy
import rospy
import roslib; roslib.load_manifest('ur_driver')
from ur_msgs.srv import SetIO
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
import tf; import tf.transformations as tft
import numpy as np


def main():

    try:
        print "Init robot (ctrl+d to exit) ..."
        move = MoveUR5(sim=False)
        move.group.set_max_velocity_scaling_factor(0.12)
        move.add_table_obstacle()
        
        print "Go to camera position (joint angles) ..."
        move.go_camera_position()
        rospy.sleep(3) 
        
        block_pos = [0.02289368,  0.10472973,  0.68946166, 0,0,0]
        
        cam_sets = (0.094, -0.031, 0.0235) # m
        WE = move.get_self_pose()
        EC = move.get_camera_transformation(cam_sets, current=False)
        CB = move.cartesian_to_pose(block_pos)
        EG = move.get_gripper_transformation(-1*pi/4)
        WB = WE.dot(EC).dot(CB)
        
        move.go_to_block([WB[0,3],WB[1,3], WB[2,3], -2*pi/4, 0, -3*pi/4])# -3*pi/4])  
        
        print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveUR5(object):
    def __init__(self, sim=False):
        super(MoveUR5, self).__init__()
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur5', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.group = moveit_commander.MoveGroupCommander('manipulator')
        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link() 
        
        self.group_names = self.robot.get_group_names()
        
        self.sim = sim # will not use gripper logic if false
        self.svcpath = '/ur_driver/set_io'
        self.back_distance = 0.15 # meters
        self.wait_between_pick_place = 2.0 # seconds
        #self.group.set_planner_id('RRTConnectkConfigDefault')
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.allow_replanning(True)
        self.group.allow_looking(True)
        self.group.set_num_planning_attempts(500)
        #self.group.set_goal_position_tolerance(0.01) # m
        #self.group.set_goal_orientation_tolerance(0.1) # rad

        if not self.sim:
            self.setioreq = SetIO()
            self.setiosvc = rospy.ServiceProxy(self.svcpath, SetIO)
            self.setioreq.fun = 1 # dIO function
            self.setioreq.pin = 0 # dIO pin
            rospy.wait_for_service(self.svcpath, 5.0)
        
    def set_constraints(self):
        self.group.clear_path_constraints()
        consts = Constraints()
        consts.joint_constraints.append(JointConstraint(joint_name='shoulder_pan_joint',
                                   position=pi/2, tolerance_above=pi/3,
                                   tolerance_below=pi/3, weight=1))
        consts.joint_constraints.append(JointConstraint(joint_name='shoulder_lift_joint',
                                   position=0, tolerance_above=0,
                                   tolerance_below=2*pi/3, weight=1))
        #consts.joint_constraints.append(JointConstraint(joint_name='elbow_joint',
        #                           position=0, tolerance_above=pi,
        #                           tolerance_bel=pi, weight=0.5))
        consts.joint_constraints.append(JointConstraint(joint_name='wrist_1_joint',
                                   position=pi/2, tolerance_above=pi/3,
                                   tolerance_below=pi/3, weight=1))
        #consts.joint_constraints.append(JointConstraint(joint_name='wrist_2_joint',
        #                           position=0, tolerance_above=0.7,
        #                           tolerance_below=pi, weight=1))
        consts.joint_constraints.append(JointConstraint(joint_name='wrist_3_joint',
                                   position=pi/4, tolerance_above=0.3,
                                   tolerance_below=0.3, weight=1))
        self.group.set_path_constraints(consts)
    
    def clear_constraints(self):
        self.group.clear_path_constraints()
    
    def print_robot_status(self, title=''):
        rospy.sleep(1.0) # allow motion to rest before printing
        print "ROBOT STATUS PRINT: %s" % title
        print "Reference frame: %s" % self.planning_frame
        print "End effector: %s" % self.eef_link
        print "Robot Groups:", self.robot.get_group_names()
        print "Printing robot state"
        print self.robot.get_current_state()
        print ""
        print "Printing robot position"
        print self.group.get_current_pose(self.eef_link)
        print ""

    def set_vacuum(self, state): 
        if not self.sim:
            self.setioreq.state = 1.0 if state else 0.0 
            try:
                resp = self.setiosvc(self.setioreq.fun, self.setioreq.pin, 
                        self.setioreq.state)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % (e,))
            return resp
        return False
    
    def go_camera_position(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = pi/2
        joint_goal[1] = 0
        joint_goal[2] = -pi/2
        joint_goal[3] = pi/2
        joint_goal[4] = -pi/2
        joint_goal[5] = pi/4
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_starting_position(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = pi/2
        joint_goal[1] = -pi/4
        joint_goal[2] = -pi/2
        joint_goal[3] = 3*pi/4
        joint_goal[4] = -pi/2
        joint_goal[5] = pi/4
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_joint_path(self, angles):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[:6] = angles[:6]
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def add_table_obstacle(self):
        self.scene.remove_world_object('back_wall')
        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = 'base_link'
        wall_pose.pose.position.y = 1.0
        self.scene.add_box('back_wall', wall_pose, size=(2.0, 0.03, 2.0))
        self.scene.remove_world_object('table')
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.z = -0.03
        self.scene.add_box('table', table_pose, size=(2.0, 2.0, 0.03))

    def add_jenga_obstacle(self):
        orient = tft.quaternion_from_euler(pi/4, 0, 0, axes='rzyx') # rotate tower by 45deg
        jenga_pose = geometry_msgs.msg.PoseStamped()
        jenga_pose.header.frame_id = 'base_link'
        jenga_pose.pose.orientation.x = orient[0]
        jenga_pose.pose.orientation.y = orient[1]
        jenga_pose.pose.orientation.z = orient[2]
        jenga_pose.pose.orientation.w = orient[3]
        jenga_pose.pose.position.z = 0.0
        jenga_pose.pose.position.y = -0.508#-0.5334
        jenga_pose.pose.position.x = -0.1016#-0.1143
        self.scene.add_box('jenga', jenga_pose, size=(0.2, 0.2, 0.5))

    def go_to_pose_goal(self, pose_goal):
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        rospy.sleep(2)
        self.group.set_start_state_to_current_state()
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
  
    def delta(self, start, end, num):
        return (end - start)/num
  
    def plan_constrained_cartesian_path(self, g_pose, interp=20):
        rospy.sleep(1) # stabilization for joint angles
        self.group.set_start_state_to_current_state()
        cur_pose = self.group.get_current_pose().pose
        px = self.delta(g_pose.position.x, cur_pose.position.x, interp)
        py = self.delta(g_pose.position.y, cur_pose.position.y, interp)
        pz = self.delta(g_pose.position.z, cur_pose.position.z, interp)
        cur_pose.orientation = g_pose.orientation
        waypoints = []        
        for _ in range(interp):
            cur_pose.position.x += -1*px
            cur_pose.position.y += -1*py
            cur_pose.position.z += -1*pz
            waypoints.append(copy.deepcopy(cur_pose))

        self.waypoints = waypoints
        (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,                  # waypoints to follow
                                           eef_step=0.05,              # eef_step
                                           jump_threshold=0.0,         # jump_threshold
                                           avoid_collisions=True)
        return plan, fraction

    def execute_plan(self, plan):
        self.group.execute(plan, wait=True)

    def go_to_block(self, goal_block, mid_path_drop=False):
        #self.add_jenga_obstacle()
        start = self.initial_pick_up_pose(goal_block)
        self.go_to_pose_goal(self.get_pose_direct(start))
        #self.scene.remove_world_object('jenga')
        
        goal = self.get_pose(*goal_block) 
        (plan, fraction) = self.plan_constrained_cartesian_path(goal)
        if fraction > 0.9:
            self.execute_plan(plan)
        else:
            print('Fraction: %d. Planning failed going backwards. Exiting.' % fraction)
            return False
            #exit(1)
        
        #self.set_vacuum(True)
        print('Start wait.')
        rospy.sleep(self.wait_between_pick_place)
        print('End wait.')
        #raw_input()
        self.set_vacuum(not mid_path_drop)
        self.print_robot_status('Final Location') 
        start = self.get_pose_direct(start)
        (plan, fraction) = self.plan_constrained_cartesian_path(start)
        if fraction > 0.9:
            self.execute_plan(plan)
        else:
            print('Fraction: %d. Planning failed going backwards. Exiting.' % fraction)
            return False
            #exit(1)
        rospy.sleep(3)
        #self.set_vacuum(False)
        self.group.clear_pose_targets()
        self.print_robot_status('Retreat Location') 
        
        return True
    # calculate initial position based on goal location
    def initial_pick_up_pose(self, goal):
        x,y,z,ry,rp,rr = goal
        bd = self.back_distance
        D_f = tft.translation_matrix((x,y,z))
        R_f = tft.euler_matrix(ry,rp,rr, 'rzyx')
        print '------------  Goal Location Transformation  ---------------'
        T_f = D_f.dot(R_f)
        print T_f
        print '-------------------- Retreat Offsets ----------------------'
        D_i = tft.translation_matrix((bd*np.cos(pi+ry),bd*np.sin(pi+ry),0))
        print D_i
        print '------------ Retreat Location Transformation --------------'
        T_i = D_i.dot(T_f)
        print T_i
        return T_i

    # get current position of robot as homogeneous transform
    def get_self_pose(self):
        pose = self.group.get_current_pose(self.eef_link).pose
        x,y,z = pose.position.x, pose.position.y, pose.position.z
        D_s = tft.translation_matrix((x,y,z))
        x,y = pose.orientation.x, pose.orientation.y
        z,w = pose.orientation.z, pose.orientation.w  
        R_s = tft.quaternion_matrix((x,y,z,w))
        return D_s.dot(R_s)
    

    def calculate_block_position(self, cam_position, current=False):
        T_block = self.cartesian_to_pose(cam_position)
        T_cam = self.get_camera_transformation(current=current)
        return T_cam.dot(T_block) # TODO: These may be flipped. IDK if this is what we want actually

    def get_gripper_transformation(self, yaw):
        # transformation for gripper
        suction_cup = 0.01 # meters, used to offset suction press
        gripper_offsets =np.array([0.0,-0.1397+suction_cup,-0.0889, 1]) 
        R = tft.rotation_matrix(pi/2+yaw, (0,0,1)) # rotation around z
        p= R.dot(gripper_offsets)
        D_e = tft.translation_matrix(p[:3])  # m
        return D_e
    
    def get_camera_transformation(self, offsets=None, current=False):
        # R is to fix axes correspondance to gripper
        Rx = tft.rotation_matrix(pi/2, (1,0,0))  
        Rz = tft.rotation_matrix(pi/2, (0,0,1))
        R = Rz.dot(Rx)

        #R_e = tft.rotation_matrix(-pi/4, (1,0,0))
        if offsets is None:
            D_e = tft.translation_matrix((0.094, -0.042, -0.031)) # m
        else:
            D_e = tft.translation_matrix(offsets) # m
        #D_e = tft.translation_matrix((0.094, -0.031, 0.042)) # m
        #D_e = tft.translation_matrix((-0.031,-0.094, -0.042)) # m
        #D_e = tft.translation_matrix((-0.031,-0.094, -0.042)) # m
        #D_e = tft.translation_matrix((0.042,-0.031, -0.094)) # m
        if current: # use current robot position to determine camera transformation
            R_e = self.get_self_pose() # use orientation of EE
            #R_e[:3,3] = 0.0 # zero-out x,y,z
        else:
            R_e = tft.rotation_matrix(-pi/4, (1,0,0)) # rotation around z
            #R_e = tft.rotation_matrix(-pi/2, (0,0,1)) # rotation around z
        
        return R_e.dot(D_e).dot(R)

    # wrt to camera and gripper transformations
    def get_pose_direct(self, goal):
        yaw, _,_ = tft.euler_from_matrix(goal, 'rzyx')
        T_e_t = self.get_gripper_transformation(yaw)
        T_c_t = self.get_camera_transformation()
        print('camera transform')
        print(T_c_t)
        T_t_g = tft.inverse_matrix(T_e_t).dot(goal)
        #T_t_g = goal        
        # TODO: camera needs rotation along x for 90 degrees
        print('final transform: (ctrl+d to exit)')
        #raw_input()
        
        #_,_,orient, pose,_ = tft.decompose_matrix(T_t_g)
        #roll,pitch,yaw = orient
        #orient = tft.quaternion_from_euler(yaw, pitch, roll, axes='rzyx')
        return self.transform_to_pose(T_t_g)

    # Convert cartesian representation to homogeneous transform
    def cartesian_to_pose(self,cart):
        x,y,z,ry,rp,rr = cart
        D_f = tft.translation_matrix((x,y,z))
        R_f = tft.euler_matrix(ry,rp,rr, 'rzyx')
        return D_f.dot(R_f)

    def transform_to_pose(self, T):
        orient = tft.quaternion_from_matrix(T)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = T[0,3] 
        pose.position.y = T[1,3]
        pose.position.z = T[2,3]
        pose.orientation.x = orient[0]
        pose.orientation.y = orient[1]
        pose.orientation.z = orient[2]
        pose.orientation.w = orient[3]
        return pose 

    # wrt to camera and gripper transformations: wrapper for get_pose_direct
    def get_pose(self,x,y,z,ry,rp,rr):
        return self.get_pose_direct(self.cartesian_to_pose((x,y,z,ry,rp,rr)))

if __name__ == '__main__':
    main()

