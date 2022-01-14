---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

{% highlight ruby %}
#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, pow



#we calculate the most optimal path by selecting the one that minimizes the distance, we can calculate the distance in 2 ways
#distance_mode=-1  #"'0' taxist mode" o '1' "euclidean mode"

#for A* we need to divide the map we want to use in a grid, the grid will then be divided in cells

#take into account that this works as a map, x positive is right axis, y positive is down axis


class Cell:  #for each cell we need some data
    def __init__(self,position, father, g, f):
        self.position=position #the position of the cell given in a '[x, y]' value
        self.father=father # the cell we come from
        self.g=g # the distance from this cell to the goal
        self.f=f # the distance travelled untill this cell

class Planner:

    def __init__(self):
        # Load map
        self.map = np.genfromtxt('./map.csv',delimiter=',')
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.resolution = 1
        
        # Print map, uncomment to print map
        image = np.flipud(self.map)
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'))
        plt.show()

    def compute_path(self, start_cell, goal_cell):
        """Compute path."""

        #declarations of list
        self.openlist=[] # here we will store all the cells we have yet to visit
        self.closedlist=[] #here we will store all the cells we have already visited
      

        #start point append to the cells to visit the one in which we start, the list gotta start somewhere
        if distance_mode==1:
            self.openlist.append(Cell([start_cell[0], start_cell[1]],[start_cell[0], start_cell[1]],0,sqrt(pow(start_cell[0]-goal_cell[0],2)+pow(start_cell[1]-goal_cell[1],2))))
        else:
            self.openlist.append(Cell([start_cell[0], start_cell[1]],[start_cell[0], start_cell[1]],0,abs(start_cell[0]-goal_cell[0])+abs(start_cell[1]-goal_cell[1])))
        
        
        
        while len(self.openlist)!=0: #while we have cells to visit

            candidate=self.openlist.pop(0) #take out one cell we want to visit
            self.closedlist.append(candidate) #put it inside the list of cells we have already visited

            if candidate.position == goal_cell: #if we have reached our goal we exit the main loop
                break

            #ok, so we have not reached the goal, lets use our 4 neighbourghs (right, left, up and down) as candidates of cells we may want to visit 

            neighbour=[[candidate.position[0]+1, candidate.position[1]], [candidate.position[0], candidate.position[1]+1], [candidate.position[0]-1, candidate.position[1]], [candidate.position[0], candidate.position[1]-1]]

            for x in range(4): #for each neighbour
                already_visited=False #we will firstly say that we have yet to visit that cell

                if self.map[neighbour[x][0], neighbour[x][1]]==0:#ok, first, are we in a wall, 0 means no wall, so we can continue
                    
                    for i in self.openlist: 
                        if ([neighbour[x][0], neighbour[x][1]] == i.position):#have we seen this cell before but we have yet to visit it?
                            already_visited=True #we will say that we have already visited it
                            if(i.g<candidate.g-1): #is it easier to go to our actual cell from where we were before? (the cost is lower)
                                candidate=self.openlist.pop() #the cost is lower, so we update the path by saying
                                candidate.father=i.position #we come from the cell whose cost is lower
                                candidate.g=i.g+1 #we update the travelled distance
                                self.openlist.append(candidate) #we store the update

                    for i in self.closedlist: 
                        if ([neighbour[x][0], neighbour[x][1]] == i.position): ##have we seen this cell before and visited it?
                            already_visited=True #we will say that we have already visited it
                            if(i.g<candidate.g-1): #is it easier to go to our actual cell from where we were before? (the cost is lower)
                                candidate=self.closedlist.pop()#the cost is lower, so we update the path by saying
                                candidate.father=i.position#we come from the cell whose cost is lower
                                candidate.g=i.g+1#we update the travelled distance
                                self.closedlist.append(candidate)#we store the update
                    
                    if already_visited==False: #seems we have yet to see this cell so lets add it to the list of cells we have yet to visit

                        #this if decides how we will measure the cost of the cell                        
                        if distance_mode==1:
                            self.openlist.append(Cell([neighbour[x][0], neighbour[x][1]],[candidate.position[0], candidate.position[1]],candidate.g+1,pow(neighbour[x][0]-goal_cell[0],2)+pow(neighbour[x][1]-goal_cell[1],2)))
                        else:
                            self.openlist.append(Cell([neighbour[x][0], neighbour[x][1]],[candidate.position[0], candidate.position[1]],candidate.g+1,abs(neighbour[x][0]-goal_cell[0])+abs(neighbour[x][1]-goal_cell[1])))

            self.openlist.sort(key=lambda Cell : Cell.f) #we sort the list by cost to restart the loop 

        path = [] #here we will store the optimal path to the goal form the start pose
        if len(self.openlist)==0 and self.closedlist[0].position!=goal_cell: #if we have tried all the paths but we didnt reach the goal position
            print("we werent able to find a path")


        else: #lets compute the path!
            parent=goal_cell #we will start with the goal cell
            while parent != start_cell: #until we get to the start point
                position=self.closedlist.pop() #we take out a cell from the list of points
                if position.position == parent: #we look for the cell whose coordinates we want
                    path.append(position.father) #we add that cell to the path
                    parent=position.father #we update the cell we are looking for

        path.reverse() #we revert the list, as we want to go from start to finish
        path.append(goal_cell)#we add the last point as it doesnt appear on the list

        
        # Print path
        
        x=[]
        y=[]
        for i in path:
            x.append(i[0])
            y.append(i[1])
        
        
        image = np.flipud(self.map)
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'))
        plt.plot(y,x,'ro-', linewidth=2, markersize=5)
        plt.show()
        
        return path

if __name__ == '__main__':
    #try:
        distance_mode=-1  #"'0' taxist mode" o '1' "euclidean mode"
        while distance_mode!=0 and distance_mode!=1:
            distance_mode=int(input("insert 1 for euclidean distance, 0 for taxists distance: "))
            print (distance_mode)
        x = Planner()
        while True:
                start_pose_x = int(input("Set your x start: "))
                start_pose_y = int(input("Set your y start: "))
                if x.map[start_pose_y,start_pose_x] == 1:
                    print("this cell is in a wall")
                else:
                    break
        while True:
                goal_pose_x = int(input("Set your x goal: "))
                goal_pose_y = int(input("Set your y goal: ")) #TODO add una secuencia de volver a pedir si la posicion esta fuera del rango del mapa, indicando dimensiones
                if x.map[goal_pose_y,goal_pose_x] == 1:
                    print("this cell is in a wall")
                else:
                    break
        
        x.compute_path([start_pose_y,start_pose_x,], [start_pose_y,goal_pose_x])
{% endhighlight %}