import numpy as np
from motion_plan import Motion_Planner
import pdb

file = open('plan', 'r')
lines = file.readlines()
plan = [line.strip() for line in lines[2:]]
# plan = ['move', 'open_drawer', 'pick_sugar', 'place_sugar', 'pick_spam', 'place_spam', 'close_drawer']
motion_planner = Motion_Planner(plan,use_gui=True)