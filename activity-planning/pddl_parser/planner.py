from .PDDL import PDDL_Parser
import numpy as np
import heapq as heap

class Node:
    def __init__(self, state):
        self.state = state
        return 
    
    def __lt__(self, other):
        if self.state[3] < other.state[3]:
            return True
        else:
            return False


class Fast_Forward_Planner:

    # -----------------------------------------------
    # Fast Forward Planner using A* as a backup planner
    # -----------------------------------------------

    def solve(self, domain, problem):
        # Parser
        parser = PDDL_Parser()
        parser.parse_domain(domain)
        parser.parse_problem(problem)
        # Parsed data
        state = parser.state
        goal_pos = parser.positive_goals
        goal_not = parser.negative_goals
        # Grounding process
        ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                ground_actions.append(act)
        # Search
        fringe = [state, None]
        while fringe:
            state = fringe.pop(0)
            plan = fringe.pop(0)
            state_heuristic = self.calculate_heuristic(ground_actions, state, goal_pos, goal_not)
            # print(state_heuristic)
            if self.applicable(state, goal_pos, goal_not):
                full_plan = []
                while plan:
                    act, plan = plan
                    full_plan.insert(0, act)
                return full_plan
            flag = True
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    if self.calculate_heuristic(ground_actions, new_state, goal_pos, goal_not) < state_heuristic:
                        fringe.append(new_state)
                        fringe.append((act, plan))
                        flag = False
                        break
            if flag:
                # print("Start BFS")
                new_state, new_plan = self.BFS(ground_actions, state, plan, goal_pos, goal_not)
                if new_state == None:
                    planner = A_star_Planner()
                    plan = planner.solve(domain, problem)
                    return plan
                fringe.append(new_state)
                fringe.append(new_plan)
    
    def BFS(self, ground_actions, state, plan, goal_pos, goal_not):
        value = self.calculate_heuristic(ground_actions, state, goal_pos, goal_not)
        visited = set([state])
        queue = [(state, plan)]
        while queue:
            state, plan = queue.pop(0)
            if self.calculate_heuristic(ground_actions, state, goal_pos, goal_not) < value:
                return state, plan
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    if new_state not in visited:
                        visited.add(new_state)
                        queue.append((new_state, (act, plan)))
        return None, None


    def calculate_heuristic(self, ground_actions, state, goal_pos, goal_not):
        current_pos = state
        current_not = state
        value = 0
        while True:
            if goal_pos.issubset(current_pos) and goal_not.isdisjoint(current_not):
                return value
            new_pos = current_pos
            new_not = current_not
            for act in ground_actions:
                if act.positive_preconditions.issubset(current_pos) and act.negative_preconditions.isdisjoint(current_not):
                    new_pos = new_pos.union(act.add_effects)
                    new_not = new_not.difference(act.del_effects)
            if current_not == new_not and current_pos == new_pos:
                break
            value += 1
            current_pos = new_pos
            current_not = new_not
        return np.inf

    def applicable(self, state, positive, negative):
        return positive.issubset(state) and negative.isdisjoint(state)

    def apply(self, state, positive, negative):
        return state.difference(negative).union(positive)


class A_star_Planner:

    # -----------------------------------------------
    # back up A* planner
    # -----------------------------------------------

    def solve(self, domain, problem):
        # Parser
        parser = PDDL_Parser()
        parser.parse_domain(domain)
        parser.parse_problem(problem)
        # Parsed data
        state = parser.state
        goal_pos = parser.positive_goals
        goal_not = parser.negative_goals
        # Grounding process
        ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                ground_actions.append(act)
        # Search
        heuristic = self.calculate_heuristic(ground_actions, state, goal_pos, goal_not)
        fringe = []
        heap.heappush(fringe, Node((state, None, 0, heuristic)))
        while fringe:
            node = heap.heappop(fringe)
            state = node.state
            state, plan, length, _= state
            if self.applicable(state, goal_pos, goal_not):
                full_plan = []
                while plan:
                    act, plan = plan
                    full_plan.insert(0, act)
                return full_plan
            flag = True
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    heuristic = self.calculate_heuristic(ground_actions, new_state, goal_pos, goal_not)
                    new_node = Node((new_state, (act, plan), length+1, length+1+heuristic))
                    heap.heappush(fringe, new_node)

    def calculate_heuristic(self, ground_actions, state, goal_pos, goal_not):
        current_pos = state
        current_not = state
        value = 0
        while True:
            if goal_pos.issubset(current_pos) and goal_not.isdisjoint(current_not):
                return value
            new_pos = current_pos
            new_not = current_not
            for act in ground_actions:
                if act.positive_preconditions.issubset(current_pos) and act.negative_preconditions.isdisjoint(current_not):
                    new_pos = new_pos.union(act.add_effects)
                    new_not = new_not.difference(act.del_effects)
            if current_not == new_not and current_pos == new_pos:
                break
            value += 1
            current_pos = new_pos
            current_not = new_not
        return np.inf

    def applicable(self, state, positive, negative):
        return positive.issubset(state) and negative.isdisjoint(state)

    def apply(self, state, positive, negative):
        return state.difference(negative).union(positive)

# -----------------------------------------------
# Main
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()
    domain = sys.argv[1]
    problem = sys.argv[2]
    verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
    planner = Fast_Forward_Planner()
    # print("Start planning")
    plan = planner.solve(domain, problem)
    # print('Time: ' + str(time.time() - start_time) + 's')
    if type(plan) is list:
        print('Length: {}'.format(len(plan)))
        print('plan:')
        for act in plan:
            print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)
