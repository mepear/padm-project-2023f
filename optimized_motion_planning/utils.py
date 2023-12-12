import numpy as np
import pdb

from pydrake.all import MathematicalProgram, Solve, Variable, eq, le, ge

def trajectory_optimization(rrt_paths):
    length = len(rrt_paths)
    
    trajectory = np.empty((7, length), dtype=Variable)
    prog = MathematicalProgram()
    for i in range(length):
        trajectory[:, i] = prog.NewContinuousVariables(7, 'variable_{}'.format(i))

    prog.AddCost(np.sum(np.subtract(2 * trajectory[:, 1:-1], trajectory[:, :-2] + trajectory[:, 2:]) ** 2))

    prog.AddConstraint(eq(trajectory[:, 0], np.array(rrt_paths[0])))
    prog.AddConstraint(eq(trajectory[:, -1], np.array(rrt_paths[-1])))
    for i in range(1, length-1):
        prog.AddConstraint(le(np.subtract(trajectory[:, i], np.array(rrt_paths[i])), 0.5 * np.ones(7)))
        prog.AddConstraint(ge(np.subtract(trajectory[:, i], np.array(rrt_paths[i])), -0.5 * np.ones(7)))
    for i in range(1, length-1):
        prog.AddConstraint(le(np.subtract(2 * trajectory[:, i], trajectory[:, i+1] + trajectory[:, i-1]), 0.1 * np.ones(7)))
        prog.AddConstraint(ge(np.subtract(2 * trajectory[:, i], trajectory[:, i+1] + trajectory[:, i-1]), -0.1 * np.ones(7)))

    result = Solve(prog)
    trajectory_sol = result.GetSolution(trajectory)
    return trajectory_sol