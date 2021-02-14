#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf2_ros
from ceng460hw1.srv import *
from ceng460hw1_utils import *
import numpy as np
from tf2_msgs.msg import TFMessage
import tf

# list to store the transformation with respect to global frame (aka treasures found)
clues_wrt_global = []
# list to store the clues found and their inverses
all_clues = []

# checks if the transformation is already in one of the lists
def check_in_list(t, l):
    for i in l:
        # compares the header and child_frame_id to existing elements in the list
        if t.header == i.header and t.child_frame_id == i.child_frame_id:
            return True
    return False

# add clues to the list all_clues or clues_wrt_global
def add_clues(resp):
    global clues_wrt_global, all_clues
    # if a treasure is successfully found
    if resp.success == True:
        for c in resp.clues:
            # if the clue is wrt to global frame
            if c.header.frame_id == "0":
                # if the clue is not already found
                if check_in_list(c, clues_wrt_global) == False:
                    # then it is a treasure, add it to clues_wrt_global
                    clues_wrt_global.append(c)
            # else add the clue to all_clues list
            else:
                # check if it is already found
                if check_in_list(c ,all_clues) == False:
                    # if not add it to the clue to all_clues list
                    all_clues.append(c)
                # also take the inverse of the clue
                inv = take_inverse(c)
                # if it not wrt global frame and it is not in the all_clues
                if inv.header.frame_id != "0" and check_in_list(inv ,all_clues) == False:
                    # add the inverse to all_clues
                    all_clues.append(inv)
                # if it is wrt global frame and it is not in the clues_wrt_global
                if inv.header.frame_id == "0" and check_in_list(inv, clues_wrt_global) == False:
                    # it is a treasure, add it to clues_wrt_global
                    clues_wrt_global.append(inv)

# find shortest angular distance
def shortest_angular_distance(from_angle, to_angle):
    # get angular difference
    angle = to_angle - from_angle
    # normalize the angle
    a = angle % (2.0*np.pi)
    # make it between -pi and pi
    if a > np.pi:
            a -= 2.0 *np.pi
    return a

# take the inverse transform of a clue
def take_inverse(a):
    # convert the transform into matrix
    m_a = TransformStamped_to_transform_matrix(a)
    # take the inverse matrix
    inv_m_a = np.linalg.inv(m_a)
    # change the child_frame_id to header_id and vice versa
    # change matrix back to transformstamped
    t_a = transform_matrix_to_TransformStamped(inv_m_a, a.child_frame_id, a.header.frame_id)
    return t_a

# if we have T_AB and T_BC -> obtain T_AC
def transform_c_to_global(g, c):
    # transform them into matrix form
    m_0h = TransformStamped_to_transform_matrix(g)
    m_hc = TransformStamped_to_transform_matrix(c)
    # multiply them
    m_0c = np.matmul(m_0h, m_hc)
    # transform the result to transformStamped
    t_0b = transform_matrix_to_TransformStamped(m_0c, g.header.frame_id, c.child_frame_id)
    return t_0b

# check if we can obtain a treasure from the clues at hand
def process_clues():
    global clues_wrt_global
    # traverse the clues T_AB
    for c in all_clues:
        # traverse the frames wrt global found T_0X
        for g in clues_wrt_global:
            # if A == X
            if c.header.frame_id == g.child_frame_id:
                # produce a new transform wrt global frame
                t_new = transform_c_to_global(g,c)
                # if it is not already in clues_wrt_global
                if check_in_list(t_new, clues_wrt_global) ==  False:
                    # add it to clues_wrt_global
                    clues_wrt_global.append(t_new)

# calculate the difference of the robots's position to goal position
def distance_to(x_current, y_current, x_goal, y_goal):
    dist = np.sqrt(((x_goal - x_current)**2)+ ((y_goal - y_current)**2))
    return dist


class Robot:

    def __init__(self):
        rospy.init_node('robot')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.zeroTime = rospy.Time()
    
    def send_diff_drive_vel_msg(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)
    
    def get_global_transformStamped(self):
        try:
            trans = self.tfBuffer.lookup_transform("odom", "base_footprint", self.zeroTime)
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    # self.move_to(x,y,0.3,3.5)
    # I tuned the P_vel to 0.3 and and P_angular to 3.5 while working on the assignment
    # While choosing these values, their ratios were critical. If the P_angular was too low compared to P_vel
    # the robot could not make smaller turns and the path became too long and the robot diverged
    # if the P_angular was too high, the robot had problems with sliding and kept making small arcs
    # if the P_vel was too high, the robot had a very fast start and I had trouble controlling its direction.
    # I also measured the time it took to find all the treasures while deciding these values
    def move_to(self, x_goal, y_goal, P_vel, P_angular, rate=20, tolerance=0.02):
        # set rate
        rate = rospy.Rate(rate)
        # get current transformation
        trans = self.get_global_transformStamped()
        # get x from the transformation read from tf2
        x = trans.transform.translation.x
        # get y from the transformation read from tf2
        y = trans.transform.translation.y
        while(distance_to(x, y, x_goal, y_goal) > tolerance):
            # calculate v*
            v_goal = P_vel * np.sqrt(((x_goal - x)**2) + ((y_goal -y)**2))
            # calculate theta*
            theta_goal = np.arctan2(y_goal - y, x_goal - x)
            # calculate current theta
            quat = [trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
            theta = tf.transformations.euler_from_quaternion(quat)
            theta_yaw = theta[2]

            # multiply theta with gain
            alpha = P_angular *  shortest_angular_distance(theta_yaw, theta_goal)

            # send command
            self.send_diff_drive_vel_msg(v_goal, alpha)
            rate.sleep()
            # get current transformation
            trans = self.get_global_transformStamped()
            # get x from the transformation read from tf2
            x = trans.transform.translation.x
            # get y from the transformation read from tf2
            y = trans.transform.translation.y

        # no more velocity, destionation reached
        self.send_diff_drive_vel_msg(0, 0)
    
    # self.rotate_to(x,y,0.7)
    # I used 0.7 for P_angular gain while testing the assignment with the given clues.
    # As I increased the P_angular, the robot rotated faster. I chose to keep a relatively small gain to better observe the behaviour
    # If P_angular is too high, it is harder to stop at the direction we want.
    def rotate_to(self, x_goal, y_goal, P_angular, rate=20, tolerance=0.02):
        rate = rospy.Rate(rate)
        # get current transformation of the robot
        trans = self.get_global_transformStamped()
        # calculate current theta (it is way component of rpy)
        quat = [trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
        theta = tf.transformations.euler_from_quaternion(quat)
        theta_yaw = theta[2]
        # get x from the transformation read from tf2
        x = trans.transform.translation.x
        # get y from the transformation read from tf2
        y = trans.transform.translation.y
        # calculate theta*
        theta_goal = np.arctan2(y_goal - y, x_goal - x)
        while(np.abs(shortest_angular_distance(theta_yaw, theta_goal)) > tolerance):
            # multiply angular diff with gain
            alpha = P_angular *  shortest_angular_distance(theta_yaw, theta_goal)
            # send command
            self.send_diff_drive_vel_msg(0, alpha)
            rate.sleep()
            # get current transformation
            trans = self.get_global_transformStamped()
            # calculate current theta
            quat = [trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
            theta = tf.transformations.euler_from_quaternion(quat)
            theta_yaw = theta[2]
            # calculate theta*
            theta_goal = np.arctan2(y_goal - y, x_goal - x)



        
    # This main is designed for normal mode, please remove self.move_to(x,y,0.3,3.5) line in easy mode
    def main(self):
        rospy.wait_for_message('tf', TFMessage)
        rospy.wait_for_service('spot_announcement')
        announce_spot = rospy.ServiceProxy('spot_announcement', SpotAnnouncement)

        first_announcement = SpotAnnouncementRequest()
        first_announcement.spot.child_frame_id = "0"
        first_resp = announce_spot(first_announcement)
        # add first and identitity clue
        first_announcement.spot.header.frame_id = "0"
        clues_wrt_global.append(first_announcement.spot)
        if not first_resp.success:
            rospy.logerr('something is wrong!')
            exit()
        rospy.loginfo('first response: %s'% first_resp)

        # process first clue
        add_clues(first_resp)
        process_clues()

        tf_transform = self.get_global_transformStamped()
        rospy.loginfo('tf_transform: %s' % tf_transform)
        rospy.loginfo('transform_matrix: %s' % TransformStamped_to_transform_matrix(tf_transform))

        treasure_count = 1
        # N < 15 and first is already received
        while(treasure_count<15):
            # if there are still treasures found
            if treasure_count < len(clues_wrt_global):
                # get coordinates of the treasure
                x = clues_wrt_global[treasure_count].transform.translation.x
                y = clues_wrt_global[treasure_count].transform.translation.y
                # move to clue
                # PLEASE REMOVE THE FOLLOWING LINE IN EASY MODE !!!!
                self.move_to(x,y,0.3,3.5)
                # self.rotate_to(x,y,0.7)
                # announce clue
                resp = announce_spot(clues_wrt_global[treasure_count])
                # add clue to lists
                add_clues(resp)
                # check if we can find any treasure from the available clues
                process_clues()
                # increment treasure count
                treasure_count+=1
            else:
                break


if __name__ == "__main__":
    try:
        robot = Robot()
        robot.main()
    except rospy.ROSInterruptException:
        pass