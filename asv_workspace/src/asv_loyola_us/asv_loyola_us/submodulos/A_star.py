#!/usr/bin/env python
import numpy as np
from math import sqrt, pow, ceil, floor
#TODO:
    #Use diagonals whenever possible
    #optimize using line of sight comparing last point with start point then next point till you can go directly
    #then use this program
    
    
#we calculate the most optimal path by selecting the one that minimizes the distance, we can calculate the distance in 2 ways
#distance_mode=-1  #"'0' taxist mode" o '1' "euclidean mode"

#for A* we need to divide the map we want to use in a grid, the grid will then be divided in cells

#take into account that this works as a map, x positive is right axis, y positive is down axis

class Cell:  #for each cell we need some data
    def __init__(self,position, father, g, f):
        self.position=position #the position of the cell given in a '[x, y]' value
        self.father=father # the cell we come from
        self.g=g # the distance from this cell to the goal
        self.f=f # the distance travelled (cost) untill this cell

class A_star:

    def __init__(self, name, distance_mode=1, resolution=2, try_detailed=False):
        # Load map
        mapa=str(name+"plantilla.csv")
        gps=str(name+"grid.csv")
        self.map = np.genfromtxt(mapa,delimiter=';')
        self.read_gps(gps)
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.resolution = resolution
        self.distance_mode=distance_mode
        if try_detailed:
            self.detailed_map=np.genfromtxt("pantilla.csv",delimiter=' ')
        
    def read_gps(self, gps):
        self.locations=[]
        with open(gps, 'r') as of:
            for line in of:
                row=[]
                for coordinate in line.split(";"):
                    if coordinate!='\n':
                        aux=coordinate.split(",")
                        coord=[float(aux[0]),float(aux[1])]
                        row.append(coord)
                self.locations.append(row)
                    
        
    def collision(self, start_cell, end_cell):
        #there are better ways, but we will just take a roundabout here to make sure we do not crash
        
        
        #lets try if movement is linear to save calculation time
        if start_cell[0] == end_cell[0]:#movimiento vertical o el punto es el mismo
            y=range(start_cell[1],end_cell[1])
            x=[start_cell[0]]*len(y)
        elif start_cell[1] == end_cell[1]: #movimiento horizontal
            x=range(start_cell[0],end_cell[0])
            y=[start_cell[1]]*len(x)
           
        #lets calculate points if movement is not linear
        else:
            number_of_points=max(abs(start_cell[0]*self.resolution-end_cell[0]*self.resolution),abs(start_cell[1]*self.resolution-end_cell[1]*self.resolution))
            x=[start_cell[0]]
            y=[start_cell[1]]
            for i in range(1,number_of_points-1):        #last point has already been checked, so no need to check it again
                x.append(x[len(x)-1]+(end_cell[0]-start_cell[0])/number_of_points)
                y.append((y[len(y)-1]+(end_cell[1]-start_cell[1])/number_of_points))
        for i in range(len(x)):
            #print([ceil(x[i]), ceil(y[i])], [floor(x[i]), floor(y[i])], self.map[ceil(x[i]), ceil(y[i])]==1 or self.map[floor(x[i]), floor(y[i])]==1)
            if self.map[ceil(x[i]), ceil(y[i])]==1 or self.map[floor(x[i]), floor(y[i])]==1: #we hit an obstacle
                return True
        return False
        
        
        
    def shorten_path(self):
        iter_gap=3
        if self.path is not None: #we have a path
            i=2
            while i <len(self.path):
                if not self.collision(self.path[i-2],self.path[i]): #if we can shorten the path from the start
                    self.path.pop(i-1)
                else:
                    i+=1
                if iter_gap!=0:
                    j=i+iter_gap
                    while j<len(self.path):
                        if not self.collision(self.path[j-2],self.path[j]): #if we can shorten the path
                            self.path.pop(j-1)
                        j+=iter_gap
                        
                
                

    def compute_path(self, start_cell, goal_cell):
        """Compute path."""
        check=False
        self.path=None #first we asume we dont have a path

        #declarations of list
        self.openlist=[] # here we will store all the cells we have yet to visit
        self.closedlist=[] #here we will store all the cells we have already visited
      

        #start point append to the cells to visit the one in which we start, the list gotta start somewhere
        if self.distance_mode==1:
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
                                candidate=self.closedlist.pop() #the cost is lower, so we update the path by saying
                                candidate.father=i.position #we come from the cell whose cost is lower
                                candidate.g=i.g+1 #we update the travelled distance
                                self.closedlist.append(candidate) #we store the update
                                #self.closedlist.append(i) #TODO: hay un bug aquí

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
                        if self.distance_mode==1:
                            self.openlist.append(Cell([neighbour[x][0], neighbour[x][1]],[candidate.position[0], candidate.position[1]],candidate.g+1,pow(neighbour[x][0]-goal_cell[0],2)+pow(neighbour[x][1]-goal_cell[1],2)))
                        else:
                            self.openlist.append(Cell([neighbour[x][0], neighbour[x][1]],[candidate.position[0], candidate.position[1]],candidate.g+1,abs(neighbour[x][0]-goal_cell[0])+abs(neighbour[x][1]-goal_cell[1])))

            self.openlist.sort(key=lambda Cell : Cell.f) #we sort the list by cost to restart the loop 

        path = [] #here we will store the optimal path to the goal from the start pose

        if len(self.openlist)==0 and self.closedlist[0].position!=goal_cell: #if we have tried all the paths but we didnt reach the goal position
            print("we werent able to find a path")
            return None
        
        else: #lets compute the path!
            self.closedlist.sort(key=lambda Cell : Cell.f) #to pop the points in a good order
            parent=goal_cell #we will start with the goal cell
            while parent != start_cell: #until we get to the start point  
                found=False
                for i in self.closedlist: #check if parent is in closed list
                    if i.position == parent:
                        path.append(i.father)
                        parent=i.father
                        found=True
                        break;
                if not found:
                    for i in self.openlist: #check if parent is in closed list
                        if i.position == parent:
                            path.append(i.father)
                            parent=i.father
                            found=True
                            break;
                if not found:
                    raise
                    
                                

        path.reverse() #we revert the list, as we want to go from start to finish
        path.append(goal_cell)#we add the last point as it doesnt appear on the list

        
        # Print path
        
        self.path=path
        #print(path)
        
        
    
    def calculate_cell(self, coordinates):
        for i in range(self.width):
            if self.locations[0][i][1]>coordinates[1]:
                startx=i
                break;
        for j in range(self.height):
            if self.locations[j][0][0]<coordinates[0]:
                starty=j
                break;
        return [starty,startx]
    
    def calculate_gps(self, cell):
        lat=(self.locations[cell[0]][cell[1]][0]+self.locations[cell[0]+1][cell[1]][0])/2
        lon=(self.locations[cell[0]][cell[1]][1]+self.locations[cell[0]][cell[1]+1][1])/2
        return [lat,lon]
    
    def compute_gps_path(self,start,destination):
        self.gps_path=[]
        start_cell=self.calculate_cell(start)
        goal_cell=self.calculate_cell(destination)
        self.compute_path(start_cell, goal_cell)
        self.pathlen=len(self.path)
        self.shorten_path()
        self.new_len=len(self.path)
        self.gps_path=[]
        for i in self.path:
            self.gps_path.append(self.calculate_gps(i))
        if len(self.gps_path)>0: #if we have points
            self.gps_path.pop()
            self.gps_path.append(destination)
            
    def between_limits(self, coordinate):
        NW=self.locations[0][0]
        SE=self.locations[self.height-1][self.width-1] #TODO: use abs values to use the whole world
        if (coordinate[0]<SE[0]) or (coordinate[0]>NW[0]) or (coordinate[1]>SE[1]) or (coordinate[1]<NW[1]):
            return False
        return True
                
            
def main():
        

    #read map
    mapp= A_star("Loyola121x239")
    start=[37.30756124092905, -5.940235027872745]
    destination=[37.308586181428446, -5.939682210759604]
    mapp.compute_gps_path(start, destination)


if __name__ == '__main__':  main()