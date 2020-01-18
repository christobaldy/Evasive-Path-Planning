#PRIMARY APP FOR SURVEILLANCE EVASION: EVASIVE PATH PLANNING
#Author: Christopher Archer (caa234@cornell.edu)
from Surveillance_Evasion import *
from maps import *

class SurvApp(GameApp):
	"""
	Instance Attributes:
		view: The game view with which to view the grid [GView, inherited from GameApp]
		input: Player provided input [GInput, inherited from GameApp]
		_grid: The grid object in which to load the information [Grid]
		_evader: the evader object to traverse the grid [Evader]
		_surveil: The surveillor object to monitor the grid [Surveillor]
		_state: Current state of the grid (also later) [int]
	"""

	def start(self):
		"""
		Starts the application
		"""
		self._grid = multiple_corridors_grid()#threecubes_grid()#multiple_corridors_grid()
		self._surveil = Surveillor(self._grid, [(15,15),(4,15),(25,15)],distribution = [0.4,0.2,0.4])#,transition = np.array([[0.9,0.1,0],[0,0.9,0.1],[0.1,0,0.9]]))#threecubes_surv(self._grid)#Surveillor(self._grid, [(15,15),(4,15),(25,15)])
		self._evader = Evader(self._grid, self._surveil)
		self._state = STATE_DIJKSTRA


	def update(self, dt):
		"""
		Updates the application 

		dt: time since last update (float)
		"""
		assert isinstance(dt,float) or isinstance(dt,int)
		if self._state == STATE_SETUP:
			
			if self.input.is_key_down('r'):
				self._evader.idx=0
				self._surveil.idx = 0
				for cell in self._evader.prob_dijk_path:
					cell.human = 0
				for cell in self._evader.dijk_path2:
					cell.human = 0
				for cell in self._evader.dijk_path3:
					cell.human= 0

				self._evader.dijk = True
				self._evader.dijk_path = True
				self._evader.prob_path = True
				self._state = STATE_DIJKSTRA

			elif self.input.is_key_down('q'):
				self._evader.idx=0
				self._surveil.idx = 0

				for cell in self._evader.prob_dijk_path:
					cell.human = 0
				for cell in self._evader.dijk_path2:
					cell.human = 0
				for cell in self._evader.dijk_path3:
					cell.human= 0

				self._evader.dijk = True
				self._evader.dijk_path = False
				self._evader.prob_path = False
				self._state = STATE_DIJKSTRA

			elif self.input.is_key_down('w'):
				self._evader.idx=0
				self._surveil.idx = 0

				for cell in self._evader.prob_dijk_path:
					cell.human = 0
				for cell in self._evader.dijk_path2:
					cell.human = 0
				for cell in self._evader.dijk_path3:
					cell.human= 0

				self._evader.dijk = False
				self._evader.dijk_path = True
				self._evader.prob_path = False
				
				self._state = STATE_DIJKSTRA

			elif self.input.is_key_down('e'):
				self._evader.idx=0
				self._surveil.idx = 0

				for cell in self._evader.prob_dijk_path:
					cell.human = 0
				for cell in self._evader.dijk_path2:
					cell.human = 0
				for cell in self._evader.dijk_path3:
					cell.human= 0

				self._evader.dijk = False
				self._evader.dijk_path = False
				self._evader.prob_path = True
				self._state = STATE_DIJKSTRA

		else:
			try:
				self._evader.updatepath(dt)
				self._surveil.updatepath(dt)
			except IndexError:
				self._evader.idx = 0
				self._surveil.idx = 0
				self._state = STATE_SETUP
		pass

	def draw(self):
		"""
		Draws the gamestate
		"""
		self._grid.draw(self.view)