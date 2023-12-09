python -B -m activity_planning.pddl_parser.planner activity_planning/kitchen/kitchen.pddl activity_planning/kitchen/pb.pddl > plan
python sample_based_motion_planning/pipeline.py
rm plan