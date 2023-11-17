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


### Execution and Results

You can simply run the algorithm on the kitchen environment as following

```
python -B -m activity-planning.pddl_parser.planner activity-planning/kitchen/kitchen.pddl activity-planning/kitchen/pb.pddl
```

This will output the plan information and the plan length.

```
Length: 5
plan:
move
open_drawer
carry_sugar_to_counter
carry_spam_to_drawer
close_drawer
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
