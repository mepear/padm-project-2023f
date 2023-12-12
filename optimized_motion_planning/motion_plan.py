from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import pdb

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])


from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, unit_from_theta,get_joint_positions, wait_for_duration, inverse_kinematics_helper, quat_from_euler, euler_from_quat, set_joint_position

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

from rrt import *
from utils import trajectory_optimization

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    # print(name)
    # pdb.set_trace()
    world.add_body(name, color=np.ones(4))
    pose = pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name, pose

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

PAUSE = 0.01
PAUSE_LONG = 2.5
class Motion_Planner():
    def __init__(self, plan, use_gui=True):
        self.plan = plan

        self.world = World(use_gui=use_gui)
        self.sugar_box, self.sugar_box_pose = add_sugar_box(self.world, idx=0, counter=1, pose2d=(0.0, 0.65, np.pi / 4))
        self.spam_box, self.spam_box_pose = add_spam_box(self.world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
        # wait_for_user()
        self.world._update_initial()
        self.tool_link = link_from_name(self.world.robot, 'panda_hand')
        self.joints = get_movable_joints(self.world.robot)
        # print('Base Joints', [get_joint_name(self.world.robot, joint) for joint in self.world.base_joints])
        # print('Joints', [get_joint_name(self.world.robot, joint) for joint in self.joints])
        # print('Arm Joints', [get_joint_name(self.world.robot, joint) for joint in self.world.arm_joints])
        self.ik_joints = get_ik_joints(self.world.robot, PANDA_INFO, self.tool_link)
        # pdb.set_trace()
        self.motion_plan()
    
    def rotate(self, step_size, step, start_pose):
        goal_xy_pos = start_pose[:2]
        theta = start_pose[-1]
        for i in range(step):
            theta += step_size
            goal_pos = (goal_xy_pos[0], goal_xy_pos[1], theta)
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos)
            wait_for_duration(PAUSE)
        return goal_pos

    def move(self, step_size, step, start_pose):
        goal_xy_pos = start_pose[:2]
        theta = start_pose[-1]
        for i in range(step):
            goal_xy_pos = goal_xy_pos + step_size* unit_from_theta(theta)
            goal_pos = (goal_xy_pos[0], goal_xy_pos[1], theta)
            # pdb.set_trace()
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos)
            wait_for_duration(PAUSE)
        return goal_pos
    
    def goto(self, goal, paths):
        start = get_joint_positions(self.world.robot, self.world.arm_joints)
        # print("Start RRT")
        # paths = rrt(self.world, start, goal)
        # print("End RRT")
        # rrt_paths = []
        # for path in paths:
        #     rrt_paths.append(np.array(path.joint_pos))
        paths = trajectory_optimization(paths)
        # for path in paths:
        #     # print(path.joint_pos)
        #     set_joint_positions(self.world.robot, self.ik_joints, path)
        #     wait_for_duration(PAUSE)
        # print(paths.shape)
        # pdb.set_trace()
        for i in range(paths.shape[1]-1):
            set_joint_positions(self.world.robot, self.ik_joints, paths[:, i+1])
            wait_for_duration(PAUSE)
    
    def goto_holding(self, goal, body, paths):
        start = get_joint_positions(self.world.robot, self.world.arm_joints)
        # paths = rrt(self.world, start, goal)
        # rrt_paths = []
        # for path in paths:
        #     rrt_paths.append(np.array(path.joint_pos))
        paths = trajectory_optimization(paths)
        # for path in paths:
        #     # print(path.joint_pos)
        #     set_pose(body, get_link_pose(self.world.robot, self.tool_link))
        #     # pose = get_link_pose(self.world.robot, self.tool_link)
        #     # euler = euler_from_quat(pose[1])
        #     # set_pose(body, (pose[0], quat_from_euler([euler[0], euler[1], euler[2] + np.pi/2])))
        #     set_joint_positions(self.world.robot, self.ik_joints, path)
        #     wait_for_duration(PAUSE)
        for i in range(paths.shape[1]-1):
            set_pose(body, get_link_pose(self.world.robot, self.tool_link))
            set_joint_positions(self.world.robot, self.ik_joints, paths[:, i+1])
            wait_for_duration(PAUSE)

    def motion_plan(self):
        for action in self.plan:
            print(action)
            if action == 'move':
                wait_for_duration(PAUSE_LONG)
                start_pose = get_joint_positions(self.world.robot, self.world.base_joints)
                goal_pos = self.rotate(-0.01, 65, start_pose)
                goal_pos = self.move(0.01, 150, goal_pos)
                goal_pos = self.rotate(0.01, 65, goal_pos)
            elif action == 'open_drawer':
                self.world.open_gripper()
                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.43, 1.18, -0.65), quat_from_euler([np.pi/2, 0, -np.pi/2])) , max_time=0.05), None)
                goal = [1.2007270551841662, -1.7595855553491675, -1.9737683465927982, -2.0879884498410597, 1.9714643899941438, 2.806091659849849, 1.6111691487577595]
                paths = np.load('rrt_paths/path_1.npy')
                self.goto(goal, paths)
                wait_for_duration(PAUSE_LONG)
                self.world.close_gripper()

                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.73, 1.18, -0.66), quat_from_euler([np.pi/2, 0, -np.pi/2])) , max_time=0.05), None)                
                goal = [0.6580787095710914, -1.1889422048343952, -2.260089028618082, -2.652050336542238, 1.4192028906945806, 2.4682067756941386, 2.120482686209028]
                print(goal)
                paths = np.load('rrt_paths/path_2.npy')
                self.goto(goal, paths)
                joint = joint_from_name(self.world.kitchen, "indigo_drawer_top_joint")
                set_joint_position(self.world.kitchen, joint, self.world.open_conf(joint))
                self.world.open_gripper()
                wait_for_duration(PAUSE_LONG)
            elif action == 'pick_sugar':
                self.world.open_gripper()
                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.14, 0.5, -0.4), quat_from_euler([0, -np.pi/2, -np.pi/4])), max_time=0.5), None)
                goal = [1.0702424300399622, 1.373856070414984, -1.1390516185175135, -0.8669791311702362, -0.5365096511713681, 2.0585505229033503, 2.5686024293470178]
                # print(get_link_pose(self.world.robot, self.tool_link))
                paths = np.load('rrt_paths/path_3.npy')
                self.goto(goal, paths)
                wait_for_duration(PAUSE_LONG)
                self.world.close_gripper()

                # # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.5, 1.08, -0.2), quat_from_euler([0, -np.pi/2, -np.pi/4])), max_time=0.5), None)
                # goal = [0.11182777236164476, -0.78167614583783, -0.5359823965802573, -2.6447530497295952, -2.3888806766143524, 2.4310207616330457, 2.8959646786533457]
                # self.goto_holding(goal, self.world.get_body("sugar_box0"))
                # wait_for_duration(PAUSE_LONG)
            elif action == 'place_sugar':
                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.1, 1., -0.3), quat_from_euler([0, -np.pi/2, -np.pi/4])), max_time=0.5), None)
                # print(goal)
                goal = [-0.5528515863785192, 0.6389534857100402, 0.606463328583921, -1.4967316645183573, -2.2767144208631773, 2.1555640462606442, 2.896974117812183]
                paths = np.load('rrt_paths/path_4.npy')
                self.goto_holding(goal, self.world.get_body("sugar_box0"), paths)
                wait_for_duration(PAUSE_LONG)
                set_pose(self.world.get_body("sugar_box0"), pose2d_on_surface(self.world, 'sugar_box0', COUNTERS[0], pose2d=(0.1, 1., -np.pi / 4)))
                self.world.open_gripper()
                wait_for_duration(PAUSE_LONG)
            elif action == 'pick_spam':
                self.world.open_gripper()
                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.32, 1.08, -0.525), quat_from_euler([0, -np.pi/2, np.pi/4])), max_time=0.5), None)
                goal = [1.3743772295259742, -1.7459460909418227, -1.8214852297816628, -2.275877766270062, -0.01017444611038787, 2.8873626152875405, -1.0802956959019316]
                paths = np.load('rrt_paths/path_5.npy')
                self.goto(goal, paths)
                wait_for_duration(PAUSE_LONG)
                self.world.close_gripper()

                # # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.5, 1.08, -0.2), quat_from_euler([0, -np.pi/2, np.pi/4])), max_time=0.5), None)
                # goal = [0.8050964269972942, -1.531907031395866, -1.1136708105333057, -2.7230443618978852, 0.06986350225494675, 2.757046445315874, -0.39220930851300784]
                # self.goto_holding(goal, self.world.get_body("potted_meat_can1"))
                # wait_for_duration(PAUSE_LONG)
            
            elif action == 'place_spam':
                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.55, 1.18, -0.3), quat_from_euler([0, -np.pi/2, np.pi/4])), max_time=0.5), None)
                print(goal)
                goal = [0.5593330125431981, -1.762743113374474, -1.4339734963269821, -2.666317441450773, -0.2294252375362298, 2.400037042000389, -0.4365209325444366]
                paths = np.load('rrt_paths/path_6.npy')
                self.goto_holding(goal, self.world.get_body("potted_meat_can1"), paths)
                wait_for_duration(PAUSE_LONG)
                set_pose(self.world.get_body("potted_meat_can1"), pose2d_on_surface(self.world, 'potted_meat_can1', 'indigo_drawer_top', pose2d=(0.5, 1.18, np.pi / 4)))
                self.world.open_gripper()
                wait_for_duration(PAUSE_LONG)
            else: #  close_drawer
                self.world.open_gripper()

                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.71, 1.18, -0.66), quat_from_euler([np.pi/2, 0, -np.pi/2])) , max_time=0.05), None)                
                goal = [0.6580787095710914, -1.1889422048343952, -2.260089028618082, -2.652050336542238, 1.4192028906945806, 2.4682067756941386, 2.120482686209028]
                print(goal)
                paths = np.load('rrt_paths/path_7.npy')
                self.goto(goal, paths)
                self.world.close_gripper()
                wait_for_duration(PAUSE_LONG)

                joint = joint_from_name(self.world.kitchen, "indigo_drawer_top_joint")
                set_joint_position(self.world.kitchen, joint, self.world.closed_conf(joint))
                set_pose(self.world.get_body("potted_meat_can1"), pose2d_on_surface(self.world, 'potted_meat_can1', 'indigo_drawer_top', pose2d=(0.15, 1.18, np.pi / 4)))
                # goal = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, ((0.48, 1.18, -0.65), quat_from_euler([np.pi/2, 0, -np.pi/2])) , max_time=0.05), None)
                goal = [1.2007270551841662, -1.7595855553491675, -1.9737683465927982, -2.0879884498410597, 1.9714643899941438, 2.806091659849849, 1.6111691487577595]
                paths = np.load('rrt_paths/path_8.npy')
                self.goto(goal, paths)
                wait_for_duration(PAUSE_LONG)
