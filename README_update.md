(place sugar box on the countertop)

# Final Project

## Activity Planning

In this part, we implement Fast Forward planner in the kitchen environment provided. The below sections will provide the detailed information about implementation.

### Source

The main folder for the project is `./activity_planning`

`./activity_planning/pddl_parser` contains all algorithm for activity planning. Below is files under the folder

- action.py (same as in [pddl-parser](https://github.com/pucrs-automated-planning/pddl-parser))
- PDDL.py (same as in [pddl-parser](https://github.com/pucrs-automated-planning/pddl-parser))
- planner.py (the Fast Forward planner we implement with A star as a backup planner)

`./activity_planning/kitchen` contains the kitchen environment in PDDL form and the given task in PDDL form

* kitchen.pddl (predicates and actions)
* pb.pddl (initial state and final task)

### Kitchen Environment

We assume that the robot can do any actions about moving the box successfully in a fixed position in front of the desk.

Also, we assume that the robot can only move the spam box into the drawer with drawer open.

We provide detailed information for the kitchen environment:

*Predicates*

**close_to_desk**: the robot is close to the desk

**sugar_on_stove**: the sugar box is on the top of the burner

**sugar_in_hand**: the sugar box is in the hand

**spam_on_counter**: the spam box is on the counter top

**spam_in_hand**: the spam box is in the hand

**sugar_on_counter**: the sugar box is on the counter top

**spam_in_drawer**: the spam box is in the drawer

**drawer_open**: the drawer is open

*Actions*

**move (move robot close to the desk)**

**open_drawer (open the drawer)**

**close_drawer (close the drawer)**

**pick_sugar (pick up the sugar box)**

**place_sugar (place sugar box on the countertop)**

**pick_spam (pick up the spam box)**

**place_spam (move the spam box in the drawer)**

*Initial condition:*

**and (sugar_on_stove) (spam_on_counter)**

*Goal:*

**and (sugar_on_counter) (spam_in_drawer) (not drawer_open)**

### Execution and Results

You can simply run the algorithm on the kitchen environment as following

```
python -B -m activity_planning.pddl_parser.planner activity_planning/kitchen/kitchen.pddl activity_planning/kitchen/pb.pddl
```

This will output the plan information and the plan length.

```
Length: 7
plan:
move 
open_drawer 
pick_sugar 
place_sugar 
pick_spam 
place_spam 
close_drawer
```

The below code can output plan into file

```
python -B -m activity-planning.pddl_parser.planner activity-planning/kitchen/kitchen.pddl activity-planning/kitchen/pb.pddl > plan
```

### Implementation Details

The planner is Fast Forward Planner with A star Planner as a backup planner

```Python
class Fast_Forward_Planner:
    def solve(self, domain, problem):
    def BFS(self, ground_actions, state, plan, goal_pos, goal_not):
    def calculate_heuristic(self, ground_actions, state, goal_pos, goal_not):
    def applicable(self, state, positive, negative):
    def apply(self, state, positive, negative):

class A_star_planner:
    def solve(self, domain, problem):
    def calculate_heuristic(self, ground_actions, state, goal_pos, goal_not):
    def applicable(self, state, positive, negative):
    def apply(self, state, positive, negative):
```

Main function for both planner is `solve()` function and it will generate the final plan for the problem. There are three same function `calculate_heuristic(), applicable(), apply()` for both planner. `calculate_heuristic()` calculates the Fast Forward heuristic for a single state. It calculate the distance of the original state with final goal on relaxed planning graph. `applicable(), apply()` are functions in [pddl-parser](https://github.com/pucrs-automated-planning/pddl-parser) planner, which calculate whether it's able to apply an action on a single state and the result state after applying an action.

`A_star_planner()` generates plan with best first search order. It will always pop a the node with minimal $h(n)+g(n)$ and generat all its child nodes.

The `Fast_Forward_planner()`  generates plan according to Enforced Hill-Climbing Algorithm. Whenever it finds a child node with less heuristic value, it goes to this node and neglect all other nodes. If there doesn't exist such node, it will use `BFS()` function to search such nodes in a BFS way. If it returns no solution, it will call `A_star_planner()` as a backup.

## Sample Based Motion Planning

In this part, we implement Sample Based Motion Planning using Rapidly exploring random trees (RRT) in the kitchen environment provided. The below sections will provide the detailed information about implementation.

### Assumptions and hardcoded values
1. We assume the sugar box and the spam box will automatically attch to the gripper after the gripper is closed
2. We assume the objects do not have collision shapes, that is, we only consider collision between robot and kitchens
3. We assume the drawer will automatically open after the `open_drawer` action and close before the `close_drawer` action
4. We hardcoded the gripper position and orientation for each stage, and then use `closest_inverse_kinematics()` to calculate joint values
5. We have fixed the probability to sample goal, distance threshold to goal, and step size for RRT

### Details
The main folder for the section is `./sample_based_motion_planning`

`./sample_based_motion_planning/pipeline.py` contains the pipeline that integrates activity plan with motion plan. It inputs plan from activity planning into `motion_plan()` in `./sample_based_motion_planning/motion_planning.py`

`./sample_based_motion_planning/motion_planning.py` contains the motion planning algorithm implementation and helper methods. Main function is `motion_plan()`. For each action, it calculates goal joint values based on hardcoded goal pose and call `goto()` or `goto_holding()` which then call `rrt()` from `./sample_based_motion_planning/rrt.py` to plan and execute trajectories. It also calls `rotate()` and `move()` to move the base. 
```Python
class Motion_Planner:
    def rotate(self, step_size, step, start_pose): # rotate base to goal orientaion
    def move(self, step_size, step, start_pose): # move base to goal position
    def goto(self, goal): # move arm to goal position and orientation
    def goto_holding(self, goal, body): # move arm to goal position and orientation, while holding objects in hand
    def motion_plan(self): # for each action, plan trajectories and execute
```

`./sample_based_motion_planning/rrt.py` contains the RRT algorithm implementation. Main function is `rrt()`. It randomly sample new node and add sampled safe nodes to the tree until reach the goal. `check_collision()`, `nearest_node()`, `add_new_node()` are helper methods. 
```Python
class TreeNode: 

def check_collision(start, new_node_pos, world): # check collision
def nearest_node(new_point, nodes): # calculate nearest node
def add_new_node(node_start, node_end): # steer
def rrt(world, start, goal): # rrt algorithm
```

### Integration of activity plan with the motion plan
As explained in previous part, `./sample_based_motion_planning/pipeline.py` contains the pipeline that integrates activity plan with motion plan. It inputs plan from activity planning into `motion_plan()` in `./sample_based_motion_planning/motion_planning.py`, which then call `rrt()` to plan trajectories and execute for each action.

### Video of robot executing the plan
Here is the link for the video of [robot executing the plan](https://drive.google.com/file/d/1YNlHLJpRRWxm-o4ysbdovcYNgG88XskH/view?usp=sharing)