PROB_CONSTANT = 0.025

#for baseline probability on empty cell
EPSILON = 0.001

#GRAPHICS STUFF
WINDOW_WIDTH =  700

WINDOW_HEIGHT = 700

#Default Grid size
DEFAULT_SIZE = 4

#Color constants
DEFAULT_COLOR = 'grey' #this represents dark grey
VISION_COLOR = 'white' 
OBS_COLOR = 'black'
SURV_COLOR = 'white'
START_COLOR = 'red'
TARGET_COLOR = 'green'
HUMAN_COLOR = 'red'
LOWER_GRADIENT = [25/255,16/255,0.25,0.999] #[59/255,67/255,113/255,1]#[13/255,20.5/255,64/255,1], [0, 12/255, 64/255,1]
UPPER_GRADIENT = [248/255, 165/255, 130/255,0.999]#[243/255,144/255,79/255,1]#[38/255,208/255,206/255,1], [240/255, 242/255, 240/255,1]
(26,41,128), (38,208,206) #dark blue and green respectively


	
SPEED = 0.05 #details number of seconds until the next path updates (speed = 0 is broken)
#States (might be useless)
STATE_SETUP = 0
STATE_DIJKSTRA = 1

