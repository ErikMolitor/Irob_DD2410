#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence


class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# tuck the arm
		b_tuck = tuckarm()	
		
		# lower head
		b_headup = movehead("up")
		b_headdown = movehead("down") 

		b_loc = localize()
		
		#pick up cube
		b_pick = pickup()
		b_place = place()

		b_detect = detect_cb()

		b_nav_pick = nav_to_pick()
		b_nav_goal = nav_to_place()

		b_e404 = respawnCube()
		b_end = counter(100, "At table?")

		loc_sq = pt.composites.Sequence(
			name="Localize seq",
			children=[b_tuck, b_headup, b_loc]
		)
		
		pick_sq = pt.composites.Sequence(
			name="Localize seq",
			children=[b_headdown, b_detect, b_pick,b_headup]
		)

		place_sq = pt.composites.Sequence(
			name="Localize seq",
			children=[b_place, b_headdown,b_detect]
		)
		

		# become the tree
		tree = pt.composites.Sequence(
		name="Main sequence",
		children=[b_e404, loc_sq, b_nav_pick, pick_sq, b_nav_goal, place_sq, b_end])

		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()




		# # drive forward 
		# b2 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(17, "At table?"), go("Go to table!", 0.5, 0)]
		# )
		# # drive forward 
		# b21 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(17, "At table?"), go("Go to table!", 0.5, 0)]
		# )

		# B44 = pt.composites.Selector(
		# 	name="Detect cube fallback",
		# 	children=[b8]
		# )


		# 		#sub programs
		# # turn 90 degree
		# b0 = pt.composites.Selector(
		# 	name="Turn 180 degrees", 
		# 	children=[counter(60, "Turn done?"), go("TURN!", 0, -0.5)]
		# )
		# b01 = pt.composites.Selector(
		# 	name="Turn 180 degrees", 
		# 	children=[counter(60, "Turn done?"), go("TURN!", 0, 0.5)]
		# )