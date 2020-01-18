import numpy as np
import collections
import math
from game2d import *
from helper_funcs import *
from consts import *
import sys
import heapq

"""
MODELS OF THE GRID AND THE ELEMENTS WITHIN IT. PRETTY MUCH EVERYTHING EXCEPT THE ACTUAL PATH PLANNING
"""

class Grid:
	#A SQUARE collection of cells with some important attributes for the game
	#learn how to implement keyword arguments
	def __init__(self, obs_map, surveil_map, size=DEFAULT_SIZE, start_cell = (3,0), target_cell=(2,3)):
		#Grid is an size x size grid of Cells corresponding to the games playing field
		#size: int size of the grid
		#start_cell: position of start cell (defaults to bottom left corner of map)
		#target_cell: position of target cell (defaults to top right corner of map)
		#density_map: an nxn array map of probable locations for surveillers (CURRENT VERSION DETERMINISTIC)
		#obs_map: set of positions of all obstacles (deterministic)
		#surveil_map: set of positions of all surveillers (can be random)
		#surveillors: LIST of all surveillors
		assert not (start_cell in obs_map or start_cell in surveil_map)
		assert (start_cell[0] >= 0 and start_cell[0] <= size-1) and (start_cell[1] >= 0 and start_cell[1] <= size-1)
		assert (target_cell[0] >= 0 and target_cell[0] <= size-1) and (target_cell[1] >= 0 and target_cell[1] <= size-1)
		assert not (target_cell in obs_map or start_cell in surveil_map)
		assert size > 1
		assert start_cell != target_cell
		assert surveil_map.isdisjoint(obs_map)

		self.size = size
		self.grid, self.start_cell, self.target_cell = self.makegrid(size,obs_map,surveil_map, start_cell, target_cell)

		#Set the ending location
		self.target_cell = self.grid[target_cell[0],target_cell[1]]
		self.target_cell.target = True

		self.surveil_map = surveil_map
		self.surveillors = []
		
		for s in surveil_map:
			self.surveillors.append(self.grid[s[0],s[1]])

		self.obs_map = obs_map
		for surv in self.surveillors:
			self.vision(surv) #create vision for all surveillors
			self.assign_prob(surv)
		#self.density_map = density_map
			


	def assign_prob(self, surv):
		"""
		Assigns the probabilities of being found at any cell
		Probability is proportional to the inverse exponential of the distance (according to constant c)
		The larger c is the more sharply the probabilities decrease 
		"""
		c = PROB_CONSTANT
		for cell in self.grid.flatten():
			#cell.prob += (1-EPSILON)*np.exp(-c*dist)/len(self.surveillors)
			cell.prob = prob_function(cell,surv)


	def makegrid(self, dim, obs_map,surveil_map, start_cell, target_cell):
		"""
		Creates and returns a square grid of cells containing the obstacles and surveillers and initializes all neighbours

		Also returns cells corresponding to the start and target cells
		
		"""
		grid = np.empty((dim,dim),dtype='O')
		cell_size = WINDOW_WIDTH//dim

		for i in range(dim):
			for j in range(dim):
				grid[i,j] = Cell((i,j),cell_size)
		
		#Set the starting location
		start_cell = grid[start_cell[0],start_cell[1]]
		start_cell.start = True
		start_cell.assign_shape(cell_size)

		#Set the ending location
		target_cell = grid[target_cell[0],target_cell[1]]
		target_cell.target = True
		target_cell.assign_shape(cell_size)

		#Set the surveiller's locations
		for s in surveil_map:
			grid[s[0],s[1]].surveil = True
			grid[s[0],s[1]].assign_shape(cell_size)

		#Set the obstacles locations
		for obs in obs_map:
			grid[obs[0],obs[1]].obstacle = True
			grid[obs[0],obs[1]].assign_shape(cell_size)
		
		#initialize neighbours as the cells adjacent to them 
		self.add_neighbours(grid,obs_map,dim)

		return grid, start_cell, target_cell
	

	def add_neighbours(self,grid,obs_map,dim):
		"""
		Adds neighbours to cells in the grid that are both valid gridpoints and are not obstacles
		"""
		#initialize neighbours as the cells adjacent to them (not diagonal)
		for i in range(dim):
			for j in range(dim):
				#iterate thru all potential neighbours, if idx out of bounds or idx is an obstactle skip
				#ADJACENT NEIGHBOURS
				try:
					if not (i+1,j) in obs_map:							
						grid[i,j].neighbours.add(grid[i+1,j])
				except IndexError:
					pass
				
				try:
					if not (i,j+1) in obs_map:
						grid[i,j].neighbours.add(grid[i,j+1])
				except IndexError:
					pass
				
				try:
					if not (i-1,j) in obs_map and i-1 >= 0:
						grid[i,j].neighbours.add(grid[i-1,j])
				except IndexError:
					pass
				
				try:
					if not (i,j-1) in obs_map and j-1 >= 0:						
						grid[i,j].neighbours.add(grid[i,j-1])
				except IndexError:
					pass
				#DIAGONAL NEIGHBOURS
				try:
					if not (i-1,j+1) in obs_map and i-1 >= 0:							
						grid[i,j].neighbours.add(grid[i-1,j+1])
				except IndexError:
					pass
				
				try:
					if not (i+1,j+1) in obs_map:
						grid[i,j].neighbours.add(grid[i+1,j+1])
				except IndexError:
					pass
				
				try:
					if not (i-1,j-1) in obs_map and i-1 >= 0 and j-1>=0:
						grid[i,j].neighbours.add(grid[i-1,j-1])
				except IndexError:
					pass
				
				try:
					if not (i+1,j-1) in obs_map and j-1 >= 0:						
						grid[i,j].neighbours.add(grid[i+1,j-1])
				except IndexError:
					pass
				

	def vision(self, s_cell):
		"""
		Returns a set of all cells that are visible from a surveillors (or potentially human's) position
		
		Vision cannot penetrate objects and is radially outward from the perspective of the surveillor
		
		Vision extends infinitely until a wall or boundary condition (defined in helper function obstacle_boundary)
		"""
		#from every object go away from the surveillor until the end of the map, this will serve as a boundary for vision
		boundary = self.obstacle_boundary(s_cell)
		self.vision_bfs(s_cell, boundary)


	def vision_bfs(self, root, boundary):
		""" 
		Adds vision map for one cell according to an obstacle boundary
		Does not return anything simply alters the cell's vision attribute
		root is the start cell
		"""
		queue = collections.deque([root])
		while queue: 
			vertex = queue.popleft()
			for neighbour in vertex.neighbours: 
				if not (neighbour in root.vision or neighbour in boundary): 
					root.vision.add(neighbour)
					neighbour.assign_shape(neighbour.shape.width,vision=True) 
					queue.append(neighbour)


	def obstacle_boundary(self,cell):
		"""
		returns the set of obstacle boundaries relative to a surveillor cell (the last thing the surveillor can see)

		First must calculate slope between surveillor and obstacle then use that as a way to decide the boundary variables

		Idea of the counter is you move in the dominant direction until the counter is one, then move in non-dominant direction
		Good way to get grid approximations for non grid slopes
		#rotate slopes by arctan(1/2d) where d is the euclidean distance between two cells
		"""
		assert cell.surveil
		boundary_set = set()
	
		for o in self.obs_map:
			obs_cell = self.grid[int(o[0]),int(o[1])]
			if obs_cell.num_neighbours() >= 4: #only care about corner blocks, to reduce computation
				spos = np.array(cell.position)
				opos = np.array(o)
				slope_vec = opos-spos
				
				if slope_vec[1] != 0:
					theta = angle_between(slope_vec,np.array([0,np.sign(slope_vec[1])]))
				else:
					theta = math.pi/2

				dist = cell.dist_euclid(obs_cell)
				objective = lambda x: np.sin(x)/np.sin((3*math.pi/4)-theta-x) - (1/(dist*math.sqrt(2))) #this was done via careful geometric calculations
				#now use it to find the x that works for rotation
				try:
					rotation_angle = bisection_search(objective)
				except:
					rotation_angle = math.pi/4
				#To account for the corners of squares, create addiational slope vectors 
				slope_vec1 = rotate(slope_vec,rotation_angle)
				slope_vec1 = np.true_divide(slope_vec1, max(abs(slope_vec1))) #now have a direction equal to 1 and one being less than one
				self.add_to_boundary(o,boundary_set,slope_vec1)

				slope_vec2 = rotate(slope_vec,-rotation_angle)
				slope_vec2 = np.true_divide(slope_vec2, max(abs(slope_vec2))) #now have a direction equal to 1 and one being less than one
				self.add_to_boundary(o,boundary_set,slope_vec2)
			
		return boundary_set


	def add_to_boundary(self,obs,boundary_set,slope_vec):
		"""
		Adds cells to boundary from a certain object according to a certain slope vector
		"""
		#note thse tuples are organized (y,x) NOT (x,y)
		#y is the dominant direction
		if abs(slope_vec[0]) > abs(slope_vec[1]):
			cur_pos = obs
			counter = slope_vec[1] #counter keeps track of position 1

			while self.is_valid_gridpoint(cur_pos):
				cur_cell = self.grid[int(cur_pos[0]),int(cur_pos[1])]
				boundary_set.add(cur_cell)
				
				if abs(counter) < 1:
					cur_pos =  (cur_pos[0]+slope_vec[0],cur_pos[1])
					counter = counter + slope_vec[1]
				else:
					cur_pos = (cur_pos[0],cur_pos[1]+np.sign(slope_vec[1]))
					counter = counter % 1

		#x is the dominant direction (includes 45 degrees) 
		else:
			cur_pos = obs
			counter = slope_vec[0] #counter keeps track of position 0

			while self.is_valid_gridpoint(cur_pos):
				cur_cell = self.grid[int(cur_pos[0]),int(cur_pos[1])]
				boundary_set.add(cur_cell)
				
				if abs(counter) < 1:
					cur_pos =  (cur_pos[0],cur_pos[1]+slope_vec[1])
					counter = counter + slope_vec[0]
				else:
					cur_pos = (cur_pos[0]+np.sign(slope_vec[0]),cur_pos[1])
					counter = counter % 1

	def is_observable(self,cell1,cell2):
		"""
		Returns true if cell 1 can see cell 2 (meaning its line of sight is not obstructed by obstacles)
		Returns false otherwise
		"""
		vision = [] #accumulator 
		cell1_pos = np.array(cell1.position)
		cell2_pos = np.array(cell2.position)
		diff_vec = cell2_pos - cell1_pos
		slope_vec=np.true_divide(diff_vec, max(abs(diff_vec)))
		
		if abs(slope_vec[0]) > abs(slope_vec[1]): #y is the dominant direction
			cur_pos = cell1.position
			counter = slope_vec[1] #counter keeps track of position 1
			i = 0
			while cur_pos != cell2.position:
				cur_cell = self.grid[int(cur_pos[0]),int(cur_pos[1])]
				vision.append(cur_cell)
				
				if abs(counter) < 1:
					if cur_pos[0] != cell2_pos[0]: 
						cur_pos =  (cur_pos[0]+slope_vec[0],cur_pos[1])
						counter = counter + slope_vec[1]
					else:
						cur_pos = (cur_pos[0],cur_pos[1]+np.sign(slope_vec[1]))
				else:
					if cur_pos[1] != cell2_pos[1]:
						cur_pos = (cur_pos[0],cur_pos[1]+np.sign(slope_vec[1]))
						counter = counter % 1
					else:
						cur_pos =  (cur_pos[0]+slope_vec[0],cur_pos[1])

				i += 1
				if i == self.size*2:
					raise AssertionError #crashes program if the target is not reached within a certain amount of time

		#x is the dominant direction (includes 45 degrees) 
		else:
			cur_pos = cell1.position
			counter = slope_vec[0] #counter keeps track of position 0
			i = 0
			while cur_pos != cell2.position:
				cur_cell = self.grid[int(cur_pos[0]),int(cur_pos[1])]
				vision.append(cur_cell)
				
				if abs(counter) < 1:
					if cur_pos[1] != cell2_pos[1]:
						cur_pos =  (cur_pos[0],cur_pos[1]+slope_vec[1])
						counter = counter + slope_vec[0]
					else:
						cur_pos = (cur_pos[0]+np.sign(slope_vec[0]),cur_pos[1])

				else:
					if cur_pos[0] != cell2_pos[0]:
						cur_pos = (cur_pos[0]+np.sign(slope_vec[0]),cur_pos[1])
						counter = counter % 1
					else:
						cur_pos =  (cur_pos[0],cur_pos[1]+slope_vec[1])
				i += 1
				if i == self.size*2:
					raise AssertionError

		for c in vision:
			if c.obstacle:
				return False

		return True


	def is_valid_gridpoint(self,point):
		"""
		returns whether point exists on the grid or not
		"""
		if point[0] < 0 or point[1] < 0: 
			return False
		
		try:
			self.grid[int(point[0]),int(point[1])]
			return True
		except:
			return False


	def draw(self, view):
		"""
		Draws the grid in a given view [GView Object]
		"""
		for row in self.grid:
			for cell in row:
				cell.shape.draw(view)
		pass


	def __repr__(self):
		return("Grid. size=%s, "%self.size + "start cell position = " + str(self.start_cell.position) + ", " + 
			"target cell position = " + str(self.target_cell.position) + ", " + 
			"number of surveillors = %f, "%int(len(self.surveil_map)) +
			"number of obstacles = %g."%len(self.obs_map))



class Cell(GRectangle):
    #Details the nodes on the map
	def __init__(self, position, cell_size, surveil=False, start=False, target=False, obstacle = False):
		#Attributes:
		#size: the size of the cell within the window [int]
		#position: shows position of node within grid [2-tuple]
		#prob (for evader): probability of being caught at this grid point: proportional to inverse square distance from surveil nodes [float in [0,1]]
		#surveil: whether or not this node is a surveiller [bool]
		#human: whether this cell had a human on it [int]
		#density (for surveiller): the probability in which there is a surveiller [float in [0,1]]
		#obstacle: if this cell is an obstacle or not [bool]
		#neighbours: The set of all neighbours a cell has [HashSet]
		#vision: The set of all nodes visible from a surveillor node [HashSet]
		#shape: the rectangle with which it is drawn
		#distance: shortest distance from the start node to the this node (value function for shortest path)
		#previous: previous node in the shortest path
		assert isinstance(cell_size, int)
		self.position = position
		self.size = cell_size

		self.surveil = surveil
		self.start = start
		self.target = target
		self.obstacle = obstacle
		self.human = 0 #This is a number not a boolean bc I want to show the densities of multiple humans

		self.prob = EPSILON
		self.density = 0

		self.neighbours = set()
		self.vision = set() #only applies to surveillors

		self.shape = GRectangle(x = cell_size//2 + cell_size * position[1], 
			y = WINDOW_HEIGHT - cell_size//2 - cell_size*position[0], height = cell_size, 
			width = cell_size, linewidth=2,fillcolor = 'grey')
		self.assign_shape(cell_size)

		self.distance = float('inf')
		self.previous = None
	

	def set_previous(self,prev_cell):
		assert isinstance(prev_cell,Cell) or prev_cell is None
		self.previous = prev_cell

	
	def get_previous(self):
		return self.previous


	def set_distance(self,dist):
		self.distance = dist

	
	def get_distance(self):
		return self.distance


	def assign_shape(self, cell_size, vision=False):
		"""
		Assigns color based on attributes and returns a GRectangle with that color
		"""
		if self.surveil:
			self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
		y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
		width = cell_size, linecolor='black',linewidth=1,fillcolor = SURV_COLOR)

		elif self.obstacle:
			self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
		y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
		width = cell_size,linewidth=1,fillcolor = OBS_COLOR)

		elif self.target:
			self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
		y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
		width = cell_size, linecolor='black',linewidth=1,fillcolor = TARGET_COLOR)

		elif self.start:
			self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
		y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
		width = cell_size, linecolor='black',linewidth=1,fillcolor = START_COLOR)

		else:
			if vision:
				self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
				y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
				width = cell_size, linecolor='black',linewidth=1,fillcolor = def_color(self.prob))
			else:
				self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
				y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
				width = cell_size, linecolor='black',linewidth=1,fillcolor = def_color(self.prob))
		
		if self.human > 0:
			self.shape = GRectangle(x = cell_size//2 + cell_size * self.position[1], 
				y = WINDOW_HEIGHT - cell_size//2 - cell_size*self.position[0], height = cell_size, 
				width = cell_size, linecolor='black',linewidth=1,fillcolor = human_color(self.human))

		pass


	def num_neighbours(self):
		return len(self.neighbours)


	def get_adj_neighbours(self):
		"""
		Returns the neighbours that share an edge with the cell
		"""
		pass

	def dist_manhattan(self,cell2):
		"""
		Returns the Manhattan distance between this cell and another on the grid
		"""
		return abs(self.position[0]-cell2.position[0])+abs(self.position[1]-cell2.position[1])

	
	def dist_euclid(self,cell2):
		"""
		Returns the Euclidean distance between this cell and another on the grid
		"""
		return ((self.position[0]-cell2.position[0])**2+(self.position[1]-cell2.position[1])**2)**0.5


	def __lt__(self,cell2):
		"""
		Returns true if the distance to cell1 is less than the distance to cell 2
		"""
		return self.get_distance() < cell2.get_distance()


	def __repr__(self):
		return("Cell. position = " + str(self.position) + ", type:" + self.start*"start, " + self.target * "target, " + 
		self.obstacle*"obstacle, " + self.surveil*"surveiller, " +
		"neighbours: %s, "%self.num_neighbours() + "prob:%d."%self.prob)