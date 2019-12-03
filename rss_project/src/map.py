#! /usr/bin/env python



import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rss_project.srv import find_frontier,find_frontierResponse

import matplotlib.pyplot as plt
import numpy as np

resolution = 0.05	# resolution of the image in m/px
offsetX = -15		# offset of the reduced map
offsetY = -15
fullSizeX = 4000	# Size in pixel of the map 4000px <-> 200m
fullSizeY = 4000
reducedSizeX = 600	# size of the map once reduced in px 600px <-> 30m
reducedSizeY = 600

# Convert the data from from the /map topic to a 2 dimensionnal array -> return only the reduced map
def convert_1D_to_2D(data1D):
	grid = np.zeros((reducedSizeX,reducedSizeY))
	x,y = fullSizeX/2-reducedSizeX/2,fullSizeY/2-reducedSizeY/2		# The starting coordinates (0,0) is at the center of the map.
	for i in range(reducedSizeX*reducedSizeY):
		grid[i%reducedSizeX][int(i/reducedSizeY)] = data1D[x+y*fullSizeX]
		x += 1
		if x >= fullSizeX/2+reducedSizeX/2:
			x = fullSizeX/2-reducedSizeX/2
			y += 1
	return grid

# Returns the occupied cells of the map
def getOccupiedCells(grid):
	x,y = [],[]
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			if grid[i][j] > 0:	# Occupied pixels has a value of 100. As this function can work with cells composed of numerous pixels, we take all the values > 0.
				x.append(i)
				y.append(j)
	return x,y

# Returns the free cells of the map
def getFreeCells(grid):
	x,y = [],[]
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			if grid[i][j] == 0:	# Free cells <-> =0
				x.append(i)
				y.append(j)
	return x,y

# Returns the unknowns cells of the map
def getUnknownedCells(grid):
	x,y = [],[]
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			if grid[i][j] < 0:	# Unknown = -1, once again, as this function can work with cells composed of numerous pixels, we take all the values < 0.
				x.append(i)
				y.append(j)
	return x,y

# Convert the unit of a map from pixel to m
# cell_size represents the number of pixel that form a cell (ex: if a cell is composed of 8x8 pixels, cell_size = 8) 
def convert_px_to_xy(x_px,y_px,cell_size):
	x = x_px * resolution*cell_size + offsetX
	y = reducedSizeY*resolution - y_px * resolution*cell_size + offsetY
	return x,y

def convert_xy_to_px(x,y,cell_size):
	x_px = int((x-offsetX)/(resolution*cell_size))
	y_px = int((y-offsetY)/(resolution*cell_size))
	return(x_px,y_px)

# Create a coast map from the complete map
# It regroups pixels to form a bigger cell, and then 
def create_coastmap(grid):
	cell_size = 8	# cell_size represents the number of pixel that form a cell (ex: if a cell is composed of 8x8 pixels, cell_size = 8)  
	coastmap = np.zeros((len(grid)/cell_size,len(grid[0])/cell_size))
	cursorX,cursorY = -1,-1
	for i in range(len(grid)):
		if i%cell_size == 0:
			cursorX += 1
		for j in range(len(grid[0])):
			if j%cell_size == 0: 
				cursorY += 1
			if cursorY >= len(coastmap):
				cursorY = 0
			coastmap[cursorX][cursorY] += grid[i][j]
	return coastmap

# Try to find the closest frontier, and the path to go to it. Not working yet.
# I will comment it later. Sorry if it is really hard to read, but you don't have to :p
def wavefront(grid,startX,startY):
	distance_map = np.zeros((len(grid),len(grid[0])))	# create the map which takes the distance values from the start point. All the non calculated cells are set at 0.
	distance_map[startX][startY] = -1	# Initialize the start at -1
	# lastDist stores the coordinates with the highest distance
	lastDistX = [startX]
	lastDistY = [startY]
	frontierFound = False	# If the frontier is found, True
	global mapCompleted 	# If there is no more cells that are not calculated, True
	global pathX,pathY
	distance = 0	# The first distance is at 0
	goalX, goalY = startX,startY
	# While there is no frontier found and the map is not complete:
	while frontierFound == False and mapCompleted == False:
		distance += 1
		# Temporary variables. Same use as lastDist
		newDistX, newDistY = [],[]
		# For each cells with the highest distance:
		for i in range(len(lastDistX)):
			# Checks the 8 cells around each cell
			for m in range(3):
				for n in range(3):
					# If a frontier is previously found, continue to the next step
					if frontierFound == True:
						continue
					else:
						# If there is a frontier: set a goal, and set frontierFound to True
						if grid[lastDistX[i]+m-1][lastDistY[i]+n-1] < 0:
							frontierFound = True
							goalX = lastDistX[i]+m-1
							goalY = lastDistY[i]+n-1
						# Else if there is an obstacle, set a really high value to the cell in the distance_map
						elif grid[lastDistX[i]+m-1][lastDistY[i]+n-1] > 0:
							distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = len(grid)*len(grid[0])+1
						# Else if there is no frontier, put the value of the distance to the cell in the distance_map, then ad the coordinate of the cell in newDist
						elif distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] == 0:
							distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = distance
							newDistX.append(lastDistX[i]+m-1)
							newDistY.append(lastDistY[i]+n-1)
		# If there is no new cell, then the map is complete
		if len(newDistX) == 0:
			mapCompleted = True
			print("Map Completed")
		# Else, we save the newDist in lastDist
		else:
			lastDistX = newDistX
			lastDistY = newDistY
	### END WHILE ###
	# Find the path to follow to reach the goal:
	pathX = [goalX]
	pathY = [goalY]
	while distance >= 2:
		distance -= 1
		nextStepFound = False
		for m in range(3):
			for n in range(3):
				if nextStepFound == True:
					continue
				elif distance_map[pathX[-1]+m-1][pathY[-1]+n-1] == distance: 
					nextStepFound = True
					pathX.append(pathX[-1]+m-1)
					pathY.append(pathY[-1]+n-1)
		#print distance
	print "-------------------------------------------------"
	#for i in range(0,70):
	#	for j in range(0,70):
	#		print int(distance_map[j][i]),
	#	print " "
	pathX.reverse()
	pathY.reverse()
	del pathX[-1]
	del pathY[-1]
	for i in range(len(pathY)):
		pathY[i] = len(grid) - pathY[i] + 1

	global endService
	endService = True
	return(pathX,pathY)

def callback(msg):
	print "start"
	grid_px = msg.data
	grid_2D = convert_1D_to_2D(grid_px)
	coastmap = create_coastmap(grid_2D)
	freeCells = getFreeCells(coastmap)
	#scaled = convert_px_to_xy(freeCells,8)

	path = wavefront(coastmap,poseX,poseY)
	for i in range(len(path[0])):
		print path[0][i],path[1][i]
		print convert_px_to_xy(path[0][i],path[1][i],8)
	print "done"

	print convert_xy_to_px(0.6,0.65,8)

	plt.scatter(freeCells[0],freeCells[1])
	#plt.scatter(occupiedCells[0],occupiedCells[1],'r')
	#plt.scatter(unknownCells[0],unknownCells[1],'b')
	plt.grid(which='major',linestyle='-', alpha=0.5)
	plt.minorticks_on()
	plt.grid(which='minor', linestyle='-', alpha=0.5)
	#plt.show()


def getPose(msg):
	global poseX
	global poseY
	poseX,poseY=convert_xy_to_px(msg.pose.pose.position.x,msg.pose.pose.position.y,8)
	 
def service_callback(request):
	doneOnce = False
	rospy.loginfo("Fontier service called")
	if doneOnce == False:
		sub = rospy.Subscriber('/map', OccupancyGrid, callback)
		sub2 = rospy.Subscriber('/odom', Odometry, getPose)
		doneOnce = True
	response = find_frontierResponse()
	print "aaa", pathX, pathY, mapCompleted
	response.complete = mapCompleted
	response.positionX = pathX
	response.positionY = pathY

	rospy.loginfo("End of service")
	return response

poseX = 0
poseY = 0
pathX = [0]
pathY = [0]
mapCompleted = False
endService = False
rospy.init_node('work_with_map', anonymous=True)
service = rospy.Service('/findfrontier', find_frontier , service_callback)

rospy.spin()
