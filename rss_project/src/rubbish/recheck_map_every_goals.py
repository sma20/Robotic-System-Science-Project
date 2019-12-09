#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from ros_ass_world.msg import the_map
import matplotlib.pyplot as plt
import numpy as np
resolution = 0.05	# resolution of the image in m/px
offsetX = -13.8#-15		# offset of the reduced map
offsetY = -15.4#-15
reducedSizeX = 544	# size of the map once reduced in px 600px <-> 30m
reducedSizeY = 544 #size of the map once reduced in px 100px<->5.2m
fullSizeX = 544	# Size in pixel of the map 4000px <-> 200m
fullSizeY = 544 #Size in pixel of the map 544px <-> 27.2m

class get_final_map():

	def __init__(self):
		self.occupiedCells= []
		self.freeCells= []
		self.unknowncells= []
		self.coastmap= []
		#self.sub = rospy.Subscriber('/ros_ass_world', the_map, self.callback)
		self.sub = rospy.Subscriber('/map', OccupancyGrid, self.callback)

		self.path=[]
		#self.pub= rospy.Publisher("/project_g",the_map,queue_size=10)


	def callback(self,msg):
		print ("on it")
		grid_px = msg.data
		grid_2D = convert_1D_to_2D(grid_px)
		coastmap = create_final_coastmap(grid_2D)
		startx=38
		starty=38
		goals=map_division(coastmap,startx,starty)
		for g in range(len(goals)):
			if g==0:
				self.path.append([move(coastmap,startx,starty,38,38)])
			else:
				self.path.append([move(coastmap,startx,starty,goals[g][0],goals[g][1])])
		

		"""
		for i in range(len(path[0])):
			print (convert_px_to_xy(path[0][i],path[1][i],8))
		print ("done")

		plt.scatter(path[0],path[1])
		#plt.scatter(occupiedCells[0],occupiedCells[1],'r')
		#plt.scatter(unknownCells[0],unknownCells[1],'b')
		plt.grid(which='major',linestyle='-', alpha=0.5)
		plt.minorticks_on()
		plt.grid(which='minor', linestyle='-', alpha=0.5)
		plt.show()
		"""

def convert_1D_to_2D(data1D):
	grid = np.zeros((reducedSizeX,reducedSizeY))
	x,y = fullSizeX/2-reducedSizeX/2,fullSizeY/2-reducedSizeY/2	#220	# The starting coordinates (0,0) is at the center of the map.
	for i in range(reducedSizeX*reducedSizeY):
		grid[i%reducedSizeX][int(i/reducedSizeY)] = data1D[x+y*fullSizeX]
		x += 1
		if x >= fullSizeX/2+reducedSizeX/2:
			x = fullSizeX/2-reducedSizeX/2
			y += 1
	return grid
	
	# Create a coast map from the complete map
# It regroups pixels to form a bigger cell, and then 
def create_final_coastmap(grid):
	cell_size = 8	# cell_size AVOUT 40cm represents the number of pixel that form a cell (ex: if a cell is composed of 8x8 pixels, cell_size = 8)  
	coastmap = np.zeros((len(grid)/cell_size,len(grid[0])/cell_size))
	cursorX,cursorY = -1,-1
	for i in range(len(grid)):
		if i%cell_size == 0:
			cursorX += 1
		for j in range(len(grid[0])):
			"""try:
				if grid[i][j]<0 and grid[i+1][j+1]<0 and grid[i+1][j]<0 and grid[i][j+1]<0:
					continue
			except IndexError:
				pass
			"""
			if j%cell_size == 0: 
				cursorY += 1
			if cursorY >= len(coastmap):
				cursorY = 0
			
			#print cursorX, cursorY
			coastmap[cursorX][cursorY] += grid[i][j]
	"""
	print("map \n")
	for i in range(len(coastmap)):#startY,goalX):
    		for j in range(len(coastmap[0])):#startX,goalX):
			print int(coastmap[i][j]),
		print(" ")
	"""
	return coastmap

def map_division(grid,startX,startY):
    
	searchx=0
	searchy=0
	goals_to_reach=[]

	#To get the extremity of the maps (not good if it's not a squered/rectangle/rond or ovale space) 
	left=0
	right=0
	up=0
	down=0

	while grid[startX-searchx][startY]>=0:
		left=startX-searchx-3 #we take one more position (unknown position), in case the map isn't perfect and starting posotion would lead to a loss of info
		searchx+=1
		
		if grid[startX-searchx][startY]<0: #To avoid stoping the process because of a thick obstacle
			if grid[startX-searchx-1][startY]>=0:
				searchx+=1	
    		
	searchx=0
	while grid[startX+searchx][startY]>=0:
		right=startX+searchx+3 #-1 because of the wall present before the unknown area
		searchx+=1

		if grid[startX+searchx][startY]<0: #To avoid stoping the process because of a thick obstacle, Simulation prob, not in real time, because the boxes have "holes"
			if grid[startX+searchx+1][startY]>=0:
				searchx+=1
	
	while grid[startX][startY-searchy]>=0:
		down=startY-searchy-3 #+1 because of the wall present before the unknown area
		searchy+=1
		
		if grid[startX][startY-searchy]<0: #To avoid stoping the process because of a thick obstacle
			if grid[startX][startY-searchy-1]>=0:
				searchx+=1
		
	searchy=0
	while grid[startX][startY+searchy]>=0:
		up=startY+searchy+3 #-1 because of the wall present before the unknown area
		searchy+=1
		if grid[startX][startY+searchy]<0: #To avoid stoping the process because of a thick obstacle (inside unknowns)
			if grid[startX][startY+searchy+1]>=0:
				searchx+=1


	xlen=(right-left)
	ylen=(up-down)
	print(right,left,up,down)
	#if ((up-down) < (left-right)): #useless actually
	map_division = np.zeros((xlen,ylen))	# (x,y)
	for x in range(left, right):
		for y in range(down,up):
    			
			
			if grid[x][y]==0 and grid[x-1][y]==0 and grid[x][y-1]==0 and grid[x-1][y-1]==0 :#and  grid[x+1][y+1]==0 and grid[x+1][y]==0 and grid[x+1][y-1]==0 and grid[x][y+1]==0 and grid[x-1][y+1]==0: #the extremities shouldn't be a problem
				#if grid[x+2][y-1]==0 and grid[x+2][y]==0 and grid[x+2][y+1]==0 and grid[x+2][y+2]==0 and grid[x+1][y+2]==0 and grid[x][y+2] ==0 and grid[x-1][y+2]==0:
					map_division[x-left][y-down]=0 
					if (len(goals_to_reach)== 0): #if there is already something in goals_to_reach, else prob 
						goals_to_reach.append([x+1, y+1])
					else:
						goal_too_close=False
						for i in range(len(goals_to_reach)):
							if ((abs((y+1)-goals_to_reach[i][1]) <2) and abs((x+1)-goals_to_reach[i][0]) <2): #if we already have a goal around this pose
								goal_too_close=True
						if (goal_too_close==False):
							goals_to_reach.append([x+1, y+1])
					#else:
    				#	 goals_to_reach.append([y+1, x+1])
				#else: 
				#	map_division[x-left][y-down]=1 #we will be missing some walls
			else:
				map_division[x-left][y-down]=1 #we will be missing some walls
	print("map division \n")		
	for i in range(xlen):#startY,goalX):
    		for j in range(ylen):#startX,goalX):
			print int(map_division[i][j]),
		print(" ")
	print("goals_to reach")
	print(goals_to_reach)
	print(np.shape(goals_to_reach))

	#If we want we can change the goals position from the closest to the further away to optimize
	#For this we do x+y of each point and get them in an order (the lowest to highest value for ex)
	

	return goals_to_reach


def move(grid,startX,startY,goalX,goalY):
	distance_map = np.zeros((len(grid),len(grid[0])))	# create the map which takes the distance values from the start point. All the non calculated cells are set at 0.
	distance_map[startX][startY] = -1	# Initialize the start at -1
	lastDistX = [startX]
	lastDistY = [startY]#[38]
	
	distance=0
	next_goal=10


	#delimit a suared zone with -1 zone as outer limit
	#area of this zone divided by x 

#not great but will do for first checks
	free_cell=True
	"""
	if (grid[goalX-1][goalY]==0): #check if prev pos x is free
		goalX=goalX-1
		free_cell=True
	elif (grid[goalX][goalY-1]==0): #else check if prev y pos is free
		goalY=goalY-1
		free_cell=True
	"""
	"""
	#else: #if none of them are free, then we go in the while to search one free, why the if and not just while, to go faster
	searchxfree=0
	searchyfree=0	
	if (grid[goalX][goalY]!=0):
		free_cell=False

	while (free_cell!=True):
		try:
			if (grid[goalX-searchxfree][goalY-searchyfree]==0):
				free_cell=True
				goalX=goalX-searchxfree
				goalY = goalY-searchyfree

			if ((searchxfree+searchyfree)%2==0):
				searchxfree+=1
			else:
				searchyfree+=1
		except IndexError:
			break
	
	while (free_cell!=True):
		try:
			if (grid[goalX+searchxfree][goalY+searchyfree]==0):
				free_cell=True
				goalX=goalX+searchxfree
				goalY = goalY+searchyfree

				if ((searchxfree+searchyfree)%2==0):
					searchxfree+=1
				else:
					searchyfree+=1		
		except IndexError:
			break
	
	if (free_cell!=True ):
		return -1 #error to ameliorate

	print(goalX,goalY)			
	"""							
	newDistX, newDistY = [],[]
	goal_found=False
	

	while goal_found != True:
		distance+=1
		# For each cells with the highest distance:
		for i in range(len(lastDistX)):
			# Checks the 8 cells around each cell
			for m in range(3):
				for n in range(3):
					if goal_found==True:
						continue
					elif lastDistX[i]+m-1 == goalX and lastDistY[i]+n-1==goalY :
						distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = distance
						newDistX.append(lastDistX[i]+m-1)
						newDistY.append(lastDistY[i]+n-1)	
						goal_found=True
						continue
					else:
						if grid[lastDistX[i]+m-1][lastDistY[i]+n-1] > 0 or grid[lastDistX[i]+m-1][lastDistY[i]+n-1] <0 : #in case we see through the arena
							distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = len(grid)*len(grid[0])+1
						#give high value to cell

						# if cell not already filled, put the value of the distance to the cell in the distance_map, then ad the coordinate of the cell in newDist
						elif distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] == 0:
							distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = distance
							newDistX.append(lastDistX[i]+m-1)
							newDistY.append(lastDistY[i]+n-1)

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
	"""
	print (distance)
	print ("-------------------------------------------------")
	for i in range(30,50):#startY,goalX):
		for j in range(30,50):#startX,goalX):
			print int(distance_map[j][i]),
		print(" ")
	"""
	pathX.reverse()
	pathY.reverse()

	return(pathX,pathY)

# cell_size represents the number of pixel that form a cell (ex: if a cell is composed of 8x8 pixels, cell_size = 8) 
def convert_px_to_xy(x_px,y_px,cell_size):
	x = x_px * resolution*cell_size + offsetX
	y = reducedSizeY*resolution - y_px * resolution*cell_size + offsetY
	return x,y

if __name__ == '__main__':
	rospy.init_node('recheck_map', anonymous=True)
	get_final_map=get_final_map()
	try:
		rospy.spin()
	#rospy.spin()
	except rospy.ROSInterruptException:
		pass
	
			