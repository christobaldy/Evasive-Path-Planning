from models import *
from functools import reduce

class Evader:
	#SANITY CHECK, SEE IF START OR FINISH IS INACCESSABLE
	def __init__(self, grid, surveil, dijk=True, dijk_path=True, prob_dijk_path=True):
		"""
		All the juicy path algorithms
		time: time since last update [float or int >= 0]
		surveil: The surveillor the evader will avoid [Surveillor]
		grid: The grid with which to implement the algos [Grid]
		dijkstra_path: The cells traversed in the shortest path using Dijkstra's algorithm [List of Cell]
		"""
		self.time = 0
		self.cell_grid = grid
		self.prob_dijk_path = []
		self.surveillor = surveil
		self.dijk = dijk
		self.dijk_path = dijk_path
		self.prob_path = prob_dijk_path

		self.idx = 0
		if dijk:
			self.prob_dijk_path, self.prob_dijk_length = self.prob_dijkstra(self.surveillor.spots,self.surveillor.distribution)
			print("Path minimizing the expectated observability from distribution "+str(self.surveillor.distribution)+". Path Length: %s"%self.prob_dijk_length)
		if prob_dijk_path:
			self.dijk_path3, self.dijk_length3 = self.prob_dijkstra_path(self.surveillor.spots,self.surveillor.distribution,self.surveillor.transition)
			print("Path minimizing the observability when given knowledge of the full surveillor path"+". Path Length: %s"%self.dijk_length3)
		if dijk_path:
			self.dijk_path2, self.dijk_length2 = self.dijkstra_path(self.surveillor.path)
			print("Path minimizing the observability when given partial knowledge of the surveillor path via stochastic transition matrix"+". Path Length: %s"%self.dijk_length2)
		
	
			

	def prob_dijkstra(self,spots,distribution, start = None, target = None):
		"""
		Finds and returns the minimum probability path from the grids start cell to the target cell

		This is the same as finding the shortest path in which all (i,j) edgeweights are equal to the observality found at the cell

		This algorithm knows the spots the surveillor might be in, and the probability of being in any of these spots, but does not know the traversal order
		As such, this algorithm minimizes the expectation of the observability at each cell
		spots: list of locations (2-tuples)
		distribution: list (or numpy array) of decimals such that it adds to one
		"""
		assert self.cell_grid.start_cell.num_neighbours() != 0, "No path exists"
		assert self.cell_grid.target_cell.num_neighbours() != 0, "No path exists"
		#assert sum(distribution) == 1 and len(distribution) == len(spots), "is not a valid probability distribution: "+str(distribution)
		# Assign the probabilities as expectation
		surv_list = [self.cell_grid.grid[i[0],i[1]] for i in spots] #list of surveillor cells
		
		if len(spots) > 1:
			for i in surv_list:
				i.surveil = True
				self.cell_grid.vision(i)
			for row in self.cell_grid.grid:
				for cell in row:
					values = [prob_function(cell,surv) for surv in surv_list]
					cell.prob = expectation(values,distribution)
			for i in surv_list:
				i.surveil= False
				i.vision = set()	
	    

		if start is None:
			start = self.cell_grid.start_cell
		if target is None:
			target = self.cell_grid.target_cell
		
		# Set the distance for the start node to zero
		start.set_distance(0)
		start.set_previous(None)

		visited = set()
		# Put tuple pair into the priority queue
		unvisited_queue = [(cell.get_distance(),cell) for cell in self.cell_grid.grid.flatten()]
		heapq.heapify(unvisited_queue)

		while len(unvisited_queue):
	    # Pops a vertex with the smallest distance 
			uv = heapq.heappop(unvisited_queue)
			current = uv[1]
			visited.add(current)

			#terminate early if target is found
			if current == target:
				break

			#for next in v.adjacent:
			for next in current.neighbours:
			    # if visited, skip
				if next in visited:
					continue
				new_dist = current.get_distance() + next.prob  #epsilon/2 is used because we dont want edweights being 0 either
			    
				if new_dist < next.get_distance():
					next.set_distance(new_dist)
					next.set_previous(current)


	        # Rebuild heap
	        # 1. Pop every item
			while len(unvisited_queue):
				heapq.heappop(unvisited_queue)
	        # 2. Put all vertices not visited into the queue
			unvisited_queue = [(cell.get_distance(),cell) for cell in self.cell_grid.grid.flatten() if not (cell in visited)]
			heapq.heapify(unvisited_queue)

	    #Makes the shortest path
		path_cell=target
		shortest_path = []
		
		while not path_cell is None:
			shortest_path.append(path_cell)
			path_cell = path_cell.get_previous()
		shortest_path.reverse()
		
		final_length = 0
		for idx,cell in enumerate(shortest_path):
			surv = self.surveillor.path[idx]
			surv.surveil=True
			self.cell_grid.vision(surv)
			final_length += prob_function(cell,surv)
			surv.vision = set()
			surv.surveil=False

		for i in range(self.cell_grid.size):
			shortest_path.append(target)
		
		return shortest_path, final_length #returns the path and path length


	def dijkstra(self, surv, start=None, target=None):
		"""
		Basic shortest path using a static Surveillor

		surv: The surveillor in which to avoid [Cell]
		start: begining of path [Cell or None]
		target: end of path [Cell or None]
		"""
		assert self.cell_grid.start_cell.num_neighbours() != 0, "No path exists"
		assert self.cell_grid.target_cell.num_neighbours() != 0, "No path exists"
		
		
		surv.surveil = True
		self.cell_grid.vision(surv)

		self.cell_grid.assign_prob(surv)


		#surv.vision = set()

		if start is None:
			start = self.cell_grid.start_cell
		if target is None:
			target = self.cell_grid.target_cell
		
		# Set the distance for the start node to zero
		for cell in self.cell_grid.grid.flatten():
			cell.set_distance(2**32) #big number
		start.set_distance(0)
		start.set_previous(None)



		visited = set()
		# Put tuple pair into the priority queue
		unvisited_queue = [(cell.get_distance(),cell) for cell in self.cell_grid.grid.flatten()]
		heapq.heapify(unvisited_queue)

		while len(unvisited_queue):
	    # Pops a vertex with the smallest distance 
			uv = heapq.heappop(unvisited_queue)
			current = uv[1]
			visited.add(current)

			#terminate early if target is found
			if current == target:
				break

			#for next in v.adjacent:
			for next in current.neighbours:
			    # if visited, skip
				if next in visited:
					continue
				new_dist = current.get_distance() + next.prob  #epsilon/2 is used because we dont want edweights being 0 either
			    
				if new_dist < next.get_distance():
					next.set_distance(new_dist)
					next.set_previous(current)
	        # Rebuild heap
	        # 1. Pop every item
			while len(unvisited_queue):
				heapq.heappop(unvisited_queue)
	        # 2. Put all vertices not visited into the queue
			unvisited_queue = [(cell.get_distance(),cell) for cell in self.cell_grid.grid.flatten() if not (cell in visited)]
			heapq.heapify(unvisited_queue)

	    #Makes the shortest path
		path_cell=target
		shortest_path = []
		while not path_cell is None:
			shortest_path.append(path_cell)
			path_cell = path_cell.get_previous()
		shortest_path.reverse()


		return shortest_path, target.get_distance() #returns the path and path length


	def dijkstra_path(self, path, start=None, target=None):
		"""
		Finds and returns the minimum observability path from the grids start cell to the target cell when the surveillor moves according to a predisposed path by the surveillor

		This is the same as finding the shortest path in which all (i,j) edgeweights are equal to prob of each cell [List of Cell]

		Note this path is a heuristic of the optimal path obtained by calculating dijkstra's at multiple update points, and algorithms like D* or LPA* can be used for more provably optimal results 
		"""
		assert self.cell_grid.start_cell.num_neighbours() != 0, "No path exists"
		assert self.cell_grid.target_cell.num_neighbours() != 0, "No path exists"
		#Set up the path as a list of cells
		path_cells = path
		change_list = [path_cells[i+1]==path_cells[i] for i in range(len(path_cells)-1)] #is false when the i+1 index has a position change
	    # Set the distance for the start node to zero
		if start is None:
			start = self.cell_grid.start_cell
		if target is None:
			target = self.cell_grid.target_cell 
		#start.set_distance(0)
		final_path = []
		final_length = 0
		shortest_path, path_length = self.dijkstra(path_cells[0]) #basic path

		current_cell = start
		index = 0
		i = 0

		while current_cell != target: 
			current_cell = shortest_path.pop(0)
			final_path.append(current_cell)
			#follows the original path until the position changes
			if not change_list[index]:
				#recompute shortest path
				shortest_path, path_length = self.dijkstra(path[index+1], start = current_cell) #computes a new shortest path relative to the new position

			index += 1
			i += 1
		
		
		for idx,cell in enumerate(final_path):
			surv = self.surveillor.path[idx]
			surv.surveil=True
			self.cell_grid.vision(surv)
			final_length += prob_function(cell,surv)
			surv.vision = set()
			surv.surveil=False

		for i in range(self.cell_grid.size):
			final_path.append(target) #add target multiple times to account for drawing errors
		
		return final_path, final_length


	def prob_dijkstra_path(self,spots, distribution, transition, start=None, target=None):
		"""
		Returns the minimum observability path when only the initial distribution and transition matrix is known. Assumes the surveillors position can be modelled as a Markov process

		This path is dynamic, so it is reliant on realizations of whether a surveillor is at a certain spot or not, then recalculates path based on that
		"""
		assert self.cell_grid.start_cell.num_neighbours() != 0, "No path exists"
		assert self.cell_grid.target_cell.num_neighbours() != 0, "No path exists"

	    # Set the distance for the start node to zero
		if start is None:
			start = self.cell_grid.start_cell
		if target is None:
			target = self.cell_grid.target_cell 

		init_distr = distribution #dummy variable to keep track of initial distribution
		final_path = []
		final_length = 0
		shortest_path, path_length = self.prob_dijkstra(spots, init_distr, start = start, target=target) #basic path from initial distribution
		index = 0
		i = 0 #tracker for some conditions
		current_cell = start
		dummy = 0
		
		while True:
	
			if current_cell == target:
				break
			
			if self.positive_realization(current_cell, dummy): #If the evader can see that a spot has a surveillor
				print('positive realization at time %s'%dummy)
				s = self.positive_realization(current_cell, dummy)
				trans_idx = spots.index(s)
				distribution = update_distribution_positive(distribution,trans_idx) #change the probability distribution on realization
				try:
					expected_time = round(1/(1-transition[trans_idx,trans_idx])) #calculate the expected time (rounded) it will take the surveillor to move from 
					stdev = ((transition[trans_idx,trans_idx])**0.5)//(1-transition[trans_idx,trans_idx]) # that spot using Geometric random variable
					
				except ZeroDivisionError:
					expected_time = self.cell_grid.size**2 #if surveillor cannot leave the spot just make it rly big
					stdev = 0

				shortest_path, path_length = self.prob_dijkstra(spots,distribution, start=current_cell)
				condition = lambda i,cur_cell: i < expected_time#the condition entails how long you should add to the final path
			
				
			elif self.negative_realization(current_cell, dummy): #if there were no positive realization but non-surveillor spots are observable
				#print('negative realization at time %d'%dummy)
				s = self.negative_realization(current_cell, dummy)
				trans_idx = spots.index(s)
				distribution = update_distribution_negative(distribution,trans_idx)
				#print('making shortest path')
				diff = abs(sum(distribution)-1)
				distribution[(trans_idx+1)%len(distribution)] = distribution[(trans_idx+1)%len(distribution)] - np.sign(sum(distribution)-1)*diff
				shortest_path, path_length = self.prob_dijkstra(spots,distribution, start=current_cell)
				#print('made shortest path')
				condition =  lambda i, cur_cell: bool(self.positive_realization(current_cell,dummy)) and not self.negative_realization(current_cell,dummy) or i < self.cell_grid.size//6
				
			else: #neither positive nor negative realization made
				current_cell = shortest_path.pop(0)
				final_path.append(current_cell)
				dummy += 1
				continue

			while condition(i,current_cell) and (current_cell != target):
				path_cell = shortest_path.pop(0)
				final_path.append(path_cell)
				dummy += 1
				current_cell = path_cell
				i += 1

	
			distribution = list(np.matmul(np.array(distribution),np.linalg.matrix_power(transition,i)))
			diff = abs(sum(distribution) -1) #some vectors are sliiightly bigger than one for whatever reason
			distribution[0] = distribution[0]-np.sign(sum(distribution)-1)*diff 
			shortest_path, path_length = self.prob_dijkstra(spots,distribution,start=current_cell)
			i=0
		
		for idx,cell in enumerate(final_path):
			surv = self.surveillor.path[idx]
			surv.surveil=True
			self.cell_grid.vision(surv)
			final_length += prob_function(cell,surv)
			surv.surveil=False
			surv.vision=set()

		return final_path, final_length

	
	def updatepath(self,dt):
		"""
		Updates the path by changing the ith index of the path list take with respect to the speed in which it updates
		"""
		#assert idx >= 0 and idx < len(self.dijkstra_path)
		if SPEED == 0:
			for cell in self.dijkstra_path:
				cell.human += 1
				cell.assign_shape(cell.size)
		else:

			self.time += dt
			if self.time >= SPEED:
				self.time -= SPEED
				if self.dijk:
					cell1 = self.prob_dijk_path[self.idx]
					cell1.human += 1
					cell1.assign_shape(cell1.size)
				if self.dijk_path:
					cell2 = self.dijk_path2[self.idx]
					cell2.human += 1
					cell2.assign_shape(cell2.size)
				if self.prob_path:
					cell3 = self.dijk_path3[self.idx]
					cell3.human += 1
					cell3.assign_shape(cell3.size)
				self.idx += 1


	def positive_realization(self,cur_cell, idx):
		"""
		Goes through all possible surveillor spots and returns the first cell it can observe has a surveillor in it (represented by the index of its path)
		Otherwise it returns False

		NOTE that while this does use the explicit surveillor path it is only to find which is the surveillor, the function still only uses partial knowledge of surveillor movements
		"""
		for s in self.surveillor.spots:
			cell = self.cell_grid.grid[s[0],s[1]]
			if self.cell_grid.is_observable(cur_cell,cell) and self.surveillor.path[idx] == cell:
				return s
		
		return False


	def negative_realization(self,cur_cell,idx):
		"""
		Goes through all possible surveillor spots and returns the first cell it can observe that has NO surveillor in it
		Returns False Otherwise
		"""
		for s in self.surveillor.spots:
			cell = self.cell_grid.grid[s[0],s[1]]
			if self.cell_grid.is_observable(cur_cell,cell) and not self.surveillor.path[idx] == cell:
				return s
		
		return False



class Surveillor:
	"""
	An instance of a surveillor, encoding path information and possibly path planning algorithms
	"""
	def __init__(self, grid, spots, distribution = None, transition = None):
		"""
		Instance attributes
		time: the time until the next element of the path is highlighted
		idx: current index of the path
		grid: Grid in which to act on [Grid]
		spots: potential spots for the surveillor [nonempty list of grid_points]
		distribution: Probabilities that the observer will be in any of these spots [list of floats of the same size as the number of spots]
			if distribution is None: every spot is equally likely
		transition: The probability of the surveillor transitioning from one spot to the next [matrix (ndarray) of floats or None]
			if transition is None: a degenerate transition will be used characterized by the identity matrix
		"""

		self.time = 0
		self.idx = 0
		self.cell_grid = grid
		self.spots = spots
		self.distribution = distribution
		if distribution is None:
			self.distribution = uniform_prob_vec(len(spots))
		self.transition = transition
		if transition is None:
			self.transition = np.identity(len(spots))
		self.path = self.build_path()
	

	def build_path(self):
		"""
		Builds the path for the surveillor to take
		"""
		path = []
		#Set up vision for all cells in path
		#else:
		tmp_distribution = self.distribution
		init_idx = np.random.choice(len(self.spots),p=tmp_distribution)
		pos_vec = update_distribution_positive(tmp_distribution,init_idx)
		
		final_T = self.cell_grid.size*4 #The max number of cells in a path is size*4 (loose bound)
		
		for i in range(final_T):
			pos_idx = pos_vec.index(1)
			idx = np.random.choice(len(self.spots), p=self.transition[pos_idx,:]) #initial position generated randomly
			pos = self.spots[idx]
			cell = self.cell_grid.grid[pos[0],pos[1]]
			pos_vec = update_distribution_positive(pos_vec,idx)
			path.append(cell)

		return path


	def updatepath(self, dt):
		"""
		Animaties 1 frame of the path the surveillor takes
		"""
		if SPEED == 0:
			pass
		else:
			self.time += dt
			if self.time >= SPEED:
				self.time -= SPEED
				for surv in (self.cell_grid.surveillors):
					surv.surveil = False
					surv.vision = set()

				self.cell_grid.surveillors =[]

				cell = self.path[self.idx]
				cell.surveil = True

				self.cell_grid.surveillors.append(cell)

				for surv in self.cell_grid.surveillors:
					self.cell_grid.vision(surv)
					self.cell_grid.assign_prob(surv)
					

				for row in self.cell_grid.grid:
					for pt in row:
						pt.assign_shape(cell.size)

				for pt in cell.vision:
					pt.assign_shape(cell.size, vision=True)

				cell.surveil=False
				self.idx += 1
