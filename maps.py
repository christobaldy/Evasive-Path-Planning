from models import *
from Surveillance_Evasion import *
"""
TEST MAPS FOR DIFFERENT SCENARIOS
"""

def multiple_corridors_obs():
	"""
	Creates the obstacles for the multiple corridors map (30 x 30)
	"""
	obs_set = set()
	for i in range(17,30):
		obs_set.add((i,1))

	for i in range(8,10):
		for j in range(4, 26):
			obs_set.add((i,j))

	for i in range(20,22):
		for j in range(4,26):
			obs_set.add((i,j))

	for i in range(2,5):
		for j in range(24,30):
			obs_set.add((j,i)) 

	for i in range(26,30):
		for j in range(24,30):
			obs_set.add((j,i))

	return obs_set

def multiple_corridors_grid():
	"""
	Returns the grid object for the multiple corridors map
	"""
	return Grid(multiple_corridors_obs(), {(15,15)}, start_cell = (29,0), target_cell = (0, 23), size=30)


def threecubes_obs():
	"""
	Returns the obstacle set for the 3 cubes map
	"""
	obs_set = set()

	for i in range(9,16):
		for j in range(9,16):
			obs_set.add((i,j))

	for i in range(3,7):
		for j in range(3,7):
			obs_set.add((i,j))

	for i in range(18,22):
		for j in range(18,22):
			obs_set.add((i,j))

	return obs_set


def threecubes_grid():
	"""
	Returns the grid for 3cubes map
	"""
	return  Grid(threecubes_obs(), {(7,18)},size=25,start_cell=(23,2),target_cell = (2,23))


def threecubes_surv(grid):
	"""
	Returns the surveillor for 3cubes map
	"""
	return Surveillor(grid, [(7,18),(6,17),(5,16),(4,15),(3,14),(2,13),(1,12),(0,11)])