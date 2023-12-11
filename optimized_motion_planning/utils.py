import numpy as np
import pdb

from pydrake.all import (Binding, Box, DiagramBuilder, DirectCollocation,
                         DirectTranscription,
                         FiniteHorizonLinearQuadraticRegulatorOptions,
                         GraphOfConvexSets, GraphOfConvexSetsOptions,
                         GurobiSolver, HPolyhedron, L2NormCost, LinearSystem,
                         LogVectorOutput,
                         MakeFiniteHorizonLinearQuadraticRegulator,
                         MakeFirstAvailableSolver, MathematicalProgram,
                         MosekSolver, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         PiecewisePolynomial, PlanarSceneGraphVisualizer,
                         Point, PointCloud, Rgba, RigidTransform,
                         RotationMatrix, SceneGraph, Simulator, Solve, Sphere,
                         StartMeshcat, TrajectorySource, Variable, eq, le, ge)

def trajectory_optimization(rrt_paths):
    length = len(rrt_paths)
    
    trajectory = np.empty((7, length), dtype=Variable)
    prog = MathematicalProgram()
    for i in range(length):
        trajectory[:, i] = prog.NewContinuousVariables(7, 'variable_{}'.format(i))

    prog.AddCost(np.sum(np.subtract(trajectory[:, 1:], trajectory[:, :-1]) ** 2))

    prog.AddConstraint(eq(trajectory[:, 0], rrt_paths[0]))
    prog.AddConstraint(eq(trajectory[:, -1], rrt_paths[-1]))
    for i in range(1, length-1):
        prog.AddConstraint(le(np.subtract(trajectory[:, i], rrt_paths[i]), 0.4 * np.ones(7)))
        prog.AddConstraint(ge(np.subtract(trajectory[:, i], rrt_paths[i]), -0.4 * np.ones(7)))

    result = Solve(prog)
    trajectory_sol = result.GetSolution(trajectory)
    return trajectory_sol