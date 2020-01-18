from Surveillance_Evasion import *
### TESTING BASICS LIKE PLACEMENT AND STUFF
#test2 = Grid({(2,2)},{(1,1)},size=6)
test1 = Grid({(0,1),(0,2)},{(1,1)})
#print(test1)
for i in test1.surveil_map:
	for j in test1.grid[i[0],i[1]].vision:
		print(j.position)


print(test1.start_cell)
print(test1.target_cell)
print(test1.grid[0,1]) #obstacle
print(test1.grid[1,1]) #surveiller
print(test1.grid[3,3]) #start cell
print(test1.grid[2,3]) #target cell 
print("TEST 1:")
print("NEIGHBOUR CHECK")
print(test1.grid[1,3].num_neighbours())
#assert test1.grid[0,0].num_neighbours() == 1 #next to an object
assert test1.grid[1,3].num_neighbours() == 3 #edge
assert test1.grid[2,1].num_neighbours() == 4 #middle
assert test1.grid[0,1].num_neighbours() == 2 #obstacle
print("passed neighbor check")

print("VALID GRIDPOINT CHECK")
assert test1.is_valid_gridpoint((1,1))
assert not test1.is_valid_gridpoint((5,4))
assert not test1.is_valid_gridpoint((6,6))
assert not test1.is_valid_gridpoint((-1,-1))
assert test1.is_valid_gridpoint((3.0,1.0))
print("passed valid gridpoint check")

print("\n TEST 2:")
test2 = Grid({(1,0),(1,1),(1,2)},{(2,1)},size=4,start_cell=(0,0),target_cell=(3,3))
#should only be able to see rows 2 and 3
print("VISION CHECK")
for i in test2.surveil_map:
	for j in test2.grid[i[0],i[1]].vision:
		#print(j.position)
		assert j.position[0] in [2,3]
print("passed vision check")
print("\n TEST 3")
#x will be the dominant direction
test3 = Grid({(0,3),(1,3)},{(3,2)},start_cell=(0,0),target_cell=(3,3), size=6)

for i in test3.surveil_map:
	for j in test3.grid[i[0],i[1]].vision:
		#print(j.position)
		pass
print("\n TEST 4:")	

test4 = Grid(set(),{(2,2)}, size=5,start_cell=(0,0),target_cell=(3,3))

print("PROBABILITY CHECK")
print("Single surveillor")
prob = lambda cell: cell.prob
grid1 = np.zeros((5,5))
for i in range(5):
	for j in range(5):
		grid1[i,j] = prob(test4.grid[i,j])
print(grid1)
print("Multiple surveillors")
test5 = Grid(set(),{(3,0),(3,6)}, size=7, start_cell=(0,0), target_cell = (1,1))
grid2 = np.zeros((7,7))
for i in range(7):
	for j in range(7):
		assert prob(test5.grid[i,j]) <= 1 and prob(test5.grid[i,j]) >= 0
		grid2[i,j] = prob(test5.grid[i,j])

print(grid2)

print("Behind Obstacles")
test6 = Grid({(1,0),(1,1),(1,2)},{(2,2)}, size=5,start_cell=(0,0),target_cell=(3,3))
grid3 = np.zeros((5,5))
for i in range(5):
	for j in range(5):
		grid3[i,j] = prob(test6.grid[i,j])
print(grid3)
print("passed probability check")

print("Angle Check")
v1 = np.array([1,1])
print("v1:" + str(angle_between(v1,np.array([0,np.sign(v1[1])]))))
v2 = np.array([1,-1])
print("v2:" + str(angle_between(v2,np.array([0,np.sign(v2[1])]))))
v3 = np.array([-1,1])
print("v3:" + str(angle_between(v3,np.array([0,np.sign(v3[1])]))))
v4 = np.array([-1,-1])
print("v4:" + str(angle_between(v4,np.array([0,np.sign(v4[1])]))))
print("passed angle check")

print("Rotation Check")

print("vector (0,1)")
v1 = np.array([0,1])
print("90 degree clockwise: " +str(rotate(v1,math.pi/2)))
print("90 degree counter-clockwise: " +str(rotate(v1,-math.pi/2)))
print("45 degree clockwise: " +str(rotate(v1,math.pi/4)))
print("45 degree counter-clockwise: " +str(rotate(v1,-math.pi/4)))
print("0.3 radian clockwise: " + str(rotate(v1,0.3)))

print("vector (1,1)")
v2 = np.array([1,1])
print("90 degree clockwise: " +str(rotate(v2,math.pi/2)))
print("90 degree counter-clockwise: " +str(rotate(v2,-math.pi/2)))
print("45 degree clockwise: " +str(rotate(v2,math.pi/4)))
print("45 degree counter-clockwise: " +str(rotate(v2,-math.pi/4)))
print("0.3 radian clockwise: " + str(rotate(v2,0.3)))
"""
TEST GRIDS:
1 Surveillor, Obstacles on one side: Grid({(8,12),(15,15),(9,15),(12,16)},{(12,12)},size=25, start_cell = (0,0), target_cell=(24,24))
1 surveillor, no obstacles: Grid(set(),{(12,12)},size=25, start_cell = (0,0), target_cell=(24,24))
Basic Large Grid: Grid(set(), set(),size=25, start_cell = (0,0), target_cell=(24,24))
Basic Small Grid: Grid(set(), set(),size=5, start_cell = (0,0), target_cell=(4,4))
Big Grid (runtime slow): Grid({(8,12),(15,15),(9,15),(12,16)},{(20,20)},size=41, start_cell = (0,0), target_cell=(39,39))
VENTI (runtime v slow): Grid({(8,12),(15,15),(9,15),(12,16)},{(30,30)},size=61, start_cell = (5,5), target_cell=(60,60))
2 surveillors, no obstacle: Grid(set(),{(12,1),(12,24)},size=25, start_cell = (0,0), target_cell=(24,24))
3 Cubes: Grid(threecubes_obs(), {(7,18)},size=25,start_cell=(23,2),target_cell = (2,23))
"""
def threecubes_obs():
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
	return Surveillor(grid,[(7,18),(6,17),(5,16),(4,15),(3,14),(2,13),(1,12),(0,11)])
"""
TEST SURVEILLOR PATHS

Basic 3-step back and forth: Surveillor(self._grid, [(12,12),(13,13),(14,14),(13,13)])
"""

