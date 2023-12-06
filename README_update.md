(place sugar box on the countertop)

# Final Project

## Activity Planning

In this part, we implement Fast Forward planner in the kitchen environment provided. The below sections will provide the detailed information about implementation.

### Source

The main folder for the project is `./activity-planning`

`./activity-planning/pddl_parser` contains all algorithm for activity planning. Below is files under the folder

- action.py (same as in [pddl-parser](https://github.com/pucrs-automated-planning/pddl-parser))
- PDDL.py (same as in [pddl-parser](https://github.com/pucrs-automated-planning/pddl-parser))
- planner.py (the Fast Forward planner we implement with A star as a backup planner)

`./activity-planning/kitchen` contains the kitchen environment in PDDL form and the given task in PDDL form

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
python -B -m activity-planning.pddl_parser.planner activity-planning/kitchen/kitchen.pddl activity-planning/kitchen/pb.pddl
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
