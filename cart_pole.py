import arcade
import numpy as np
import time
import sys

# Constants for drawing
FPS = 60
CART_HEIGHT = 50
CART_WIDTH = 100
WHEEL_RAD = 15
BALL_RAD = 10
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
GROUND_HEIGHT = 200
SCREEN_TITLE = "Cart-Pole"

# System constants
m = 6
M = 15
g = 0.5
l = 175
k = 1
u_max = M * 0.2

# PID Constatns
kp = 100
ki = 2
kd = 30

# Linearized system
A = np.array([
	[0, 1, 0, 0],
	[0, -k/M, -(m*g)/M, 0],
	[0, 0, 0, 1],
	[0, k/(M*l), ((M+m)*g)/(M*l), 0]
])
b = np.array([[0], [1/M], [0], [-1/(M*l)]])

# Define variables for the Cart-Pole System
states = np.array([[
	0,			# Position
	0,			# Velocity
	0.1,		# Angle
	0,			# Angular Velocity
	0			# Integral of Angle
]])
u = 0

# Calculates and returns the next state based on the current state
# and applied input. Has option for linear and non-linear state 
# variable calculation.
# INPUTS:
# 	cur_state - (4, ) np.array representing our current state
# 	u - float that represents the force applied on the system 
# 	linearized - boolean for choosing linear or non-linear system
# OUTPUTS:
# 	next_state - (4, ) np.array representing our next state
def next_state(cur_state, u, linearized=False):
	# Unpack state variables
	x = cur_state[0]
	x_dot = cur_state[1]
	theta = cur_state[2]
	theta_dot = cur_state[3]
	theta_sum = cur_state[4]

	if linearized:
		# If we are linearized, then just use Ax + bu for next state
		_, acceleration, _, ang_acceleration = A @ cur_state[:4] + b.flatten() * u
	else:
		# Calculate angular and linear acceleration
		acceleration = ((u/m) + (np.sin(theta) * l * theta_dot ** 2) - (g * np.sin(theta) * np.cos(theta)) -  \
			(x_dot * k / m)) / ((M/m) + (np.sin(theta)) ** 2)
		ang_acceleration = ((-np.cos(theta) * u / m) - (np.sin(theta) * np.cos(theta) * l * theta_dot ** 2) +  \
			((1 + M/m) * g * np.sin(theta)) + (k * x_dot * np.cos(theta)/m)) / ((M/m) + (np.sin(theta)) ** 2) / l
		
	# Update angular and linear velocities 
	vel_new = x_dot + acceleration
	ang_vel_new = theta_dot + ang_acceleration

	# Update angular and linear positions
	pos_new = x + vel_new
	ang_new = theta + ang_vel_new

	# Update integral of angle
	theta_sum_new = theta_sum + ang_new

	# Make sure angle is alwaus between -pi and pi
	while ang_new < -np.pi:
		ang_new += 2 * np.pi
	while (ang_new > np.pi):
		ang_new -= 2 * np.pi

	# Make sure the position is on the screen
	if pos_new > SCREEN_WIDTH // 2:
		pos_new -= SCREEN_WIDTH
	elif pos_new < -SCREEN_WIDTH // 2:
		pos_new += SCREEN_WIDTH

	# Return next state
	next_state = np.array([pos_new, vel_new, ang_new, ang_vel_new, theta_sum_new]) 
	return next_state

# Uses the arcade graphic functions to draw a cart pole 
# system given the current state variables.
# INPUTS:
# 	cur_state - (4, ) np.array representing our current state
# 	u - float that represents the force applied on the system 
# 	vis_inp - boolean for visualizing the input force
def draw_current_state(cur_state, u, vis_inp=False):
	# Unpack state variables
	x = cur_state[0] + SCREEN_WIDTH//2
	theta = cur_state[2]

	# Draw cart
	arcade.draw_rectangle_filled(x, GROUND_HEIGHT + 2*WHEEL_RAD + CART_HEIGHT//2, CART_WIDTH, CART_HEIGHT, arcade.color.BLUSH)
	# Draw ground
	arcade.draw_rectangle_filled(SCREEN_WIDTH//2, GROUND_HEIGHT//2, SCREEN_WIDTH, GROUND_HEIGHT, arcade.color.CYBER_GRAPE)
	# Draw wheels
	arcade.draw_circle_filled(x - CART_WIDTH // 3, GROUND_HEIGHT + WHEEL_RAD, WHEEL_RAD, arcade.color.BLACK)
	arcade.draw_circle_filled(x + CART_WIDTH // 3, GROUND_HEIGHT + WHEEL_RAD, WHEEL_RAD, arcade.color.BLACK)
	# Compute pole locations
	pole_pivot_y = GROUND_HEIGHT + CART_HEIGHT + 2*WHEEL_RAD
	pole_tip_y = pole_pivot_y + l * np.cos(theta)
	pole_tip_x = x + l * np.sin(theta)
	# Draw pole
	arcade.draw_line(x, pole_pivot_y, pole_tip_x, pole_tip_y, arcade.color.BLACK, 2)
	# Draw small mass
	arcade.draw_circle_filled(pole_tip_x, pole_tip_y, BALL_RAD, arcade.color.BLACK)
	# Visualize input
	if vis_inp:
		max_width = 200 * u_max / M
		center_height = GROUND_HEIGHT + CART_HEIGHT//2 + 2*WHEEL_RAD
		margin_height = 30
		arcade.draw_line(x, center_height, x + 200 * u / M, center_height, arcade.color.BLACK, 2)
		arcade.draw_line(x - max_width,center_height - margin_height, x - max_width, center_height + margin_height, arcade.color.RED, 2)
		arcade.draw_line(x + max_width,center_height - margin_height, x + max_width, center_height + margin_height, arcade.color.RED, 2)

# Visualizes the angle over time
def viz_angle():
	# Determine number of samples to graph (min is used for beggining stage really)
	N = min(500, states.shape[0])

	# Generate x axis
	x = np.linspace(SCREEN_WIDTH//2 - N/2, SCREEN_WIDTH//2 + N/2, N).reshape((-1,1))
	
	# Extract last N angles
	angles = 10 * 180 * states[-N:,2].reshape((-1,1)) / np.pi 
	# Keep them between -100 and 100 (i.e. -10 to 10 degrees)
	angles = angles * (np.abs(angles) < 100)
	# Shift up to top of screen
	angles += 4 * SCREEN_HEIGHT // 5

	# Append x and y into 1 tall matrix
	pairs = np.hstack((x, angles))
	# Plot the matrix and the zero line
	arcade.draw_line(SCREEN_WIDTH//2 - N/2 -10, 4 * SCREEN_HEIGHT // 5, SCREEN_WIDTH//2 + N/2 + 10, 4 * SCREEN_HEIGHT // 5, arcade.color.BLACK, 1)
	arcade.draw_line_strip(pairs, arcade.color.BLUE, 2)


# Computes an input based on the current state.
# INPUTS:
# 	cur_state - (4, ) np.array representing our current state
# RETURNS:
# 	u - float that represents the force applied on the system 
def update_input(cur_state):
	# Unpack state variables
	theta = cur_state[2]
	theta_dot = cur_state[3]
	theta_sum = cur_state[4]

	# Get desired input 
	u_desired = kp * theta  + ki * theta_sum + kd * theta_dot

	# Check for max 
	if abs(u_desired) > u_max:
		u_desired = u_max * np.sign(u_desired) 

	# Return control stategy
	return u_desired

# Built in arcade function that is called every _delta_time seconds.
def draw(_delta_time):
	global states, u, count
	arcade.start_render()

	# Extract current state
	cur_state = states[-1]

	# Respond to current state
	u = update_input(cur_state)

	# Draws the current state
	draw_current_state(cur_state, u, True)

	# Visualizes the evolution of the angle with time
	viz_angle()

	# Obtain next state
	states = np.vstack((states, next_state(cur_state, u, False)))


def main():
	global states
	arcade.open_window(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
	arcade.set_background_color(arcade.color.LAVENDER)
	arcade.schedule(draw, 1 / FPS)
	arcade.run()
	arcade.close_window()

if __name__ == "__main__":
	if len(sys.argv) == 1:
		main()
	elif len(sys.argv) == 4:
		kp = float(sys.argv[1])
		ki = float(sys.argv[2])
		kd = float(sys.argv[3])
		main()
	else:
		print("Error: Invalid number of arguments")