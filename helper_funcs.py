#File detailing the helper functions
import numpy as np
import math
from consts import *
from models import *

def unique(list1): 
	"""
	Returns the unique elemnts of a list
	"""
	# intilize a null list 
	unique_list = [] 
      
	# traverse for all elements 
	for x in list1: 
		# check if exists in unique_list or not 
		if x not in unique_list: 
			unique_list.append(x) 
	return unique_list


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ 
    Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)

    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def bisection_search(f,a=0,b=math.pi/4,N=11):
	"""
	Returns x: an approximation of f(x) = 0 when f(a)<0 and f(b)>0 after N iterations

	a,b default set to 0 and 45 degrees in order to find the angle of rotation for boundary set calculations

	Default iterations N=11 chosen to ensure an error of below 0.01 according to the formula for complexity (found online)
	"""

	assert f(a)*f(b) < 0, "Invalid Bounds"
	
	a_n = a
	b_n = b
	
	for n in range(1,N+1):
		m_n = (a_n + b_n)/2
		f_m_n = f(m_n)
		
		if f(a_n)*f_m_n < 0:
			a_n = a_n
			b_n = m_n
		
		elif f(b_n)*f_m_n < 0:
			a_n = m_n
			b_n = b_n
		
		elif f_m_n == 0: #found exact solution
			return m_n
		
		else: #Bisection method fails
			return None
	return (a_n + b_n)/2


def rotate(v,theta):
	"""
	Rotates 2-dimensional numpy array v by theta (radians) clockwise and returns the new vector
	Use the rotation matrix to help in calculations
	"""
	rot_matrix = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
	return np.matmul(v,rot_matrix)


def prob_function(cell,surv):
	"""
	Returns the probability assigned for a cell from a given surveillor (proportional to the inverser square distance)
	"""
	c = PROB_CONSTANT
	dist = surv.dist_euclid(cell)
	if cell in surv.vision:
		return math.e**(-c*dist**2)+EPSILON*1.5#c/((dist**2)+0.1)+EPSILON
	return EPSILON


def expectation(values,distribution):
	"""
	Returns the expectation of discrete random variables that takes values over some probability distribution
	distribution must be a probability vector with the same size as values
	"""
	#assert sum(distribution) == 1 and len(distribution) == len(values)
	
	expectation = 0
	for i in range(len(values)):
		expectation += values[i]*distribution[i]

	return expectation


def uniform_prob_vec(n):
	"""
	Returns a uniform probability list of size n
	i.e if n=3, this function returns [1/3,1/3,1/3]
	"""
	assert n >= 1
	vec = [1/n for i in range(n)]
	
	return vec

def update_distribution_negative(distribution,idx):
	"""
	Updates a probability vector after a realization that a certain index has prob 0
	Uses conditional probability, notably P(A|B) = P(A and B)/P(B)
	"""
	assert idx in set(range(len(distribution)))
	
	p = distribution[idx]
	new_distribution = [i/(1-p) for i in distribution]
	new_distribution[idx] = 0
	
	return new_distribution


def update_distribution_positive(distribution,idx):
	"""
	Updates a probability vector after a realization that a certain index has prob 1
	No need for conditional probability
	"""
	assert idx in set(range(len(distribution)))
	new_distribution = [0 for i in range(len(distribution))]
	new_distribution[idx]=1
	return new_distribution

#TESTING THINGS

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


def def_color(prob):
	"""
	returns the default color according to the observability at that cell
	"""
	max_prob = 1
	if prob == EPSILON:
		return [i/1.5 for i in LOWER_GRADIENT[:3]]+[1]
	prob = prob/max_prob
	color = []
	for i in range(4):
		element = (1-prob)*LOWER_GRADIENT[i]+prob*UPPER_GRADIENT[i]
		color.append(element)
	return color

def human_color(human):
	"""
	Returns the color for paths walked on (right now the color is white)
	"""
	return None#[200/255,126/255,1,0.9]

