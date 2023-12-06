import numpy as np
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, single_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, unit_from_theta,get_joint_positions, wait_for_duration, set_renderer

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
import pdb

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

class TreeNode():
    def __init__(self, joint_pos, parent):
        self.parent = parent
        self.joint_pos = joint_pos


GOAL_PROB = 0.5
THRESHOLD = 0.4

def check_collision(start, new_node_pos, world):
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    set_joint_positions(world.robot, ik_joints, new_node_pos)
    if pairwise_collision(world.robot, world.kitchen):# or single_collision(world.robot):
        return True
    set_joint_positions(world.robot, ik_joints, start)
    return False


def nearest_node(new_point, nodes):
    #TODO: accelerate
    new_point = np.array(new_point)
    min_distance = np.inf
    nearest = nodes[0]
    nearest_n = nodes[0]
    for node in nodes:
        node_pos = np.array(node.joint_pos)
        distance = np.linalg.norm(node_pos - new_point)
        if distance < min_distance:
            min_distance = distance
            nearest = node_pos
            nearest_n = node
    return nearest, nearest_n

def add_new_node(node_start, node_end):
    node_start = np.array(node_start)
    node_end = np.array(node_end)
    dist = np.linalg.norm(node_start - node_end)
    path = (node_end - node_start) / dist * min(dist, 0.3)
    return node_start + path
    
def rrt(world, start, goal):
    set_renderer(False)
    nodes = [TreeNode(start, None)]
    while True:
        random = np.random.rand()
        new_point = goal if random < GOAL_PROB else get_sample_fn(world.robot, world.arm_joints)()
        nearest, nearest_n = nearest_node(new_point, nodes)
        new_node = add_new_node(nearest, new_point)
        if not check_collision(start, new_node, world):
            new_node = TreeNode(new_node, nearest_n)
            nodes.append(new_node)
            # pdb.set_trace()
            if np.linalg.norm(new_node.joint_pos -goal) < THRESHOLD:
                backtrack = new_node
                break
    output = [backtrack]
    while backtrack is not None:
        # print(backtrack)
        output.append(backtrack.parent)
        backtrack = backtrack.parent
    output.reverse()
    # pdb.set_trace()
    set_renderer(True)
    return output[1:]