#!/usr/bin/env python
from ceng460_hw2_environment import *
import ceng460_hw2_utils
import numpy as np
from rrt_base import *
import tf.transformations
from geometry_msgs.msg import Pose, Transform, PoseStamped

CFREE_JOINT_MIN = -np.pi
CFREE_JOINT_MAX = np.pi

class RRTStar6D(GoalBiasedGreedySteerKNeighborhoodRRTStarBase):

    def __init__(self, seed):
        '''Feel free to put additional things here.'''
        GoalBiasedGreedySteerKNeighborhoodRRTStarBase.__init__(self,seed)
        self.hw2 = Ceng460Hw2Environment()

    def allclose(self,c1, c2):
        return np.allclose(c1,c2)

    # no idea if this is correct
    # calculate distance using angular difference
    def distance(self, c1, c2):
        angular_diff = self.ang_diff_subtract(c1,c2)
        # calculate eucladien distance
        d = np.sqrt(np.sum(angular_diff**2))


        return d

    # angular difference for array
    def ang_diff_subtract(self, c1, c2):
        j0 = ceng460_hw2_utils.angdiff(c1[0], c2[0])
        j1 = ceng460_hw2_utils.angdiff(c1[1], c2[1])
        j2 = ceng460_hw2_utils.angdiff(c1[2], c2[2])
        j3 = ceng460_hw2_utils.angdiff(c1[3], c2[3])
        j4 = ceng460_hw2_utils.angdiff(c1[4], c2[4])
        j5 = ceng460_hw2_utils.angdiff(c1[5], c2[5])

        arr = np.array([j0, j1, j2, j3, j4, j5])
        return arr

    def sample(self, p):
        if not self.is_goal_reachable() and self.random.random_sample() < p:
            return self.root.goal
        else:
            # sample between -pi, pi as they are joint limits in urdf
            return self.random.uniform(low=CFREE_JOINT_MIN, high=CFREE_JOINT_MAX, size=(6,))
    
    # takes joint values as numpy array
    def valid(self, c):
        joint_values = c.tolist()
        return self.hw2.check_collision(joint_values)

    # did not do anything new except for replacing np.substract by angular difference
    def collision_free(self, c1, c2, step_size):
        '''returns True if the linear trajectory between c1 and c2 are collision free.'''
        valid_c1 = self.valid(c1)
        valid_c2 = self.valid(c2)
        if valid_c1 == False or valid_c2 == False:
            return False
        if self.allclose(c1, c2) == False:
            # calculate lambda
            l = step_size/self.distance(c1,c2)
            ct = np.add(c1, np.multiply(l, self.ang_diff_subtract(c2,c1)))
            # keep steering until colision
            while self.valid(ct):
                if self.distance(ct, c2) < step_size:
                    valid_c = self.valid(c2)
                    return valid_c
                ct = np.add(ct, np.multiply(l, self.ang_diff_subtract(c2,ct)))
            return False
        else:
            return True

    # did not do anything new except for replacing np.substract by angular difference
    def steer(self, c0, c, step_size):
        '''starting from the configuration c0, gets closer to
        c in discrete steps of size step_size in terms of distance(c0, c). returns the last no-collision
        configuration. If no collisions are hit during steering, returns c2. If
        steering is not possible at all, returns None.'''
        if self.distance(c0, c) < step_size:
            valid_c = self.valid(c)
            if valid_c:
                return c
            else:
                return None
        # calculate lambda
        l = step_size/self.distance(c0,c)
        prev = c0
        ct = np.add(c0, np.multiply(l, self.ang_diff_subtract(c,c0)))
        # keep steering until colision
        while self.valid(ct):
            if self.distance(ct, c) < step_size:
                valid_c = self.valid(c)
                if valid_c:
                    return c
                else:
                    return None
            prev = ct
            ct = np.add(ct, np.multiply(l, self.ang_diff_subtract(c,ct)))
        
        return prev
        

if __name__=="__main__":

    # was already here
    rrt = RRTStar6D(460)
    rrt.hw2.move_joints((0,-0.3,0,0,0,np.pi/2))

    init_joint_values = rrt.hw2.get_current_joint_values()

    # calculate hardcoded attachment pose
    # This corresponds to Pose of object wrt ee_link
    attachment_pose = Pose()
    attachment_pose.position.x = 0.1
    quaternion = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi/2)
    attachment_pose.orientation.x = quaternion[0]
    attachment_pose.orientation.y = quaternion[1]
    attachment_pose.orientation.z = quaternion[2]
    attachment_pose.orientation.w = quaternion[3]
    # calculate Pose of ee_link wrt object
    attachment_matrix = ceng460_hw2_utils.Pose_to_matrix(attachment_pose)
    attachment_matrix_inv = tf.transformations.inverse_matrix(attachment_matrix)

    # get Pose of cube wrt world
    cube_pose = rrt.hw2.get_object_global_pose()
    cube_matrix = ceng460_hw2_utils.Pose_to_matrix(cube_pose)

    # calculate pose of end effector wrt world to attach
    ee_wrt_world = np.matmul(cube_matrix, attachment_matrix_inv)
    pose_ee_wrt_world = ceng460_hw2_utils.matrix_to_Pose(ee_wrt_world)
    
    # calculate joint values for that end effector pose
    ik = rrt.hw2.compute_ik(pose_ee_wrt_world, "world")

    # feed them to rrt*
    c_init = np.array(init_joint_values)
    c_goal = np.array(ik[0])
    p = 0.5
    k = 1
    step_size = 0.1
    rrt.init_rrt(c_init, c_goal)

    # create tree
    NUM_NODES = 50
    initial_path_found = False
    for i in range(NUM_NODES):
        rrt.add_node(p,k,step_size)
        if rrt.is_goal_reachable():
            initial_path_found = True
            break

    if initial_path_found:
        goal_edges = rrt.get_path_to_goal()

        for e in goal_edges:
            tup = tuple(e[1])
            rrt.hw2.move_joints(tup)

        # attach object
        rrt.hw2.attach()

    # get final object pose wrt world
    obj_final = rrt.hw2.final_object_pose
    obj_final_matrix = ceng460_hw2_utils.Pose_to_matrix(obj_final)

    # calculate final end effector pose
    ee_link_final = np.matmul(obj_final_matrix, attachment_matrix_inv)
    final_ee_wrt_world = ceng460_hw2_utils.matrix_to_Pose(ee_link_final)

    

    current_joint_values = rrt.hw2.get_current_joint_values()
    # calculate joint values for final end effector pose
    ik_final = rrt.hw2.compute_ik(final_ee_wrt_world, "world")

    # feed them to rrt*
    c_init = np.array(current_joint_values)
    c_goal = np.array(ik_final[0])
    rrt.init_rrt(c_init, c_goal)

    # create rrt* for final destination
    NUM_NODES = 50
    for i in range(NUM_NODES):
        rrt.add_node(p,k,step_size)
        # start going when goal is reached
        if rrt.is_goal_reachable():
            goal_edges = rrt.get_path_to_goal()
            # move in path
            for e in goal_edges:
                tup = tuple(e[1])
                rrt.hw2.move_joints(tup)

            # detach cube
            rrt.hw2.detach()

            # declare success
            rrt.hw2.declare_success()
            break



        

        


