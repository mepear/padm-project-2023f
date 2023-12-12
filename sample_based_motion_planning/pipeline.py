import numpy as np
from motion_plan import Motion_Planner
import pdb

file = open('plan', 'r')
lines = file.readlines()
plan = [line.strip() for line in lines[2:]]
motion_planner = Motion_Planner(plan,use_gui=True)