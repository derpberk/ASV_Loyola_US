
# modified from the scripts created by Hayden Eskriett [http://eskriett.com] and jbndlr
# Find the associated blog post at: http://blog.eskriett.com/2013/07/19/downloading-google-maps/
# but it has been done in a better way using the data in https://developers.google.com/maps/documentation/javascript/coordinates
# and https://stackoverflow.com/questions/40342355/how-can-i-generate-a-regular-geographic-grid-using-python





###################################IMPORTANT##################################
#Google maps works with [Latitude Longitude] variables however
#Dronekit and other programs (such as gridder) work with [Longitude Latitude] variables
##############################################################################

from PIL import Image
import os
import math
import pyproj
import matplotlib.pyplot as plt
import numpy as np
from A_star import Planner

class CustomError(Exception):
     pass

class Plan:
    
    def __init__(self, image, image_plantilla, mapa, grid):
        """
            Args:
                image:              image printed in create grid
                image_plantilla:    black and white image photoshoped from image (black=wall) (white=path)
                mapa:               csv indicating 0 for path 1 for wall
                grid:               The [Lat Lon] points of the grid as well as the size, generated in create grid.py
        """
        
        
        
        self.read_from_file(grid) #read GPS references
        self.img=Image.open(image) #open map
        self.plantilla=Image.open(image_plantilla) #open masked image
        self.planner=Planner(mapa, 1)
        
        image1 = self.img.convert("RGBA") #convert both images to RGBA
        image2 = self.plantilla.convert("RGBA")
        
        alphaBlended1 = Image.blend(image1, image2, alpha=.6) #blend images
            
        alphaBlended1.show() #show to check mask
        
    def read_from_file(self, name):
        with open(name, 'r') as of:
            of.readline()
            aux=of.readline().strip()
            self.width,self.height=aux.split(";")
            of.readline()
            self.NW_corner=of.readline().strip()
            self.NW_corner=self.NW_corner.split(";")
            self.NW_corner[0]=float(self.NW_corner[0])
            self.NW_corner[1]=float(self.NW_corner[1])
            of.readline()
            self.SE_corner=of.readline().strip()
            self.SE_corner=self.SE_corner.split(";")
            self.SE_corner[0]=float(self.SE_corner[0])
            self.SE_corner[1]=float(self.SE_corner[1])
            self.calculate_corner_pixel()
            
            
    def goto(self, start, destination):
        aux=self.to_pixel(start)
        start_pose_y = int(aux[1])
        start_pose_x = int(aux[0])
        aux = self.to_pixel(destination)
        goal_pose_y = int(aux[1])
        goal_pose_x = int(aux[0])
        
        if self.planner.map[start_pose_y, start_pose_x] == 1:
            raise CustomError("start point is in an obstacle")
        if self.planner.map[goal_pose_y, goal_pose_x] == 1:
            raise CustomError("destination point is in an obstacle")

        #print origin and destination
        image = self.planner.map
        plt.figure()
        plt.imshow(self.img, cmap=plt.get_cmap('binary'))
        plt.plot(start_pose_x,start_pose_y,'rx', markersize=5)
        plt.plot(goal_pose_x,goal_pose_y,'ro', markersize=5)
        plt.show()
            
        self.planner.compute_path([start_pose_y, start_pose_x], [goal_pose_y, goal_pose_x])
        print("path found")
        
        x=[]
        y=[]
        for i in self.planner.path:
            x.append(i[0])
            y.append(i[1])
        
        image = self.planner.map
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'))
        plt.plot(y,x,'b-', linewidth=0.5)
        plt.show()
    
        
        
        self.planner.shorten_path()
        
        x=[]
        y=[]
        for i in self.planner.path:
            x.append(i[0])
            y.append(i[1])
        
        image = self.planner.map
        plt.figure()
        plt.imshow(image, cmap=plt.get_cmap('binary'))
        plt.plot(y,x,'b-', linewidth=0.5)
        plt.show()
        
        x=[]
        y=[]
        for i in self.planner.path:
            x.append(i[0])
            y.append(i[1])
        

        image = self.planner.map
        plt.figure()
        plt.imshow(self.img, cmap=plt.get_cmap('binary'))
        plt.plot(y,x,'bo-', linewidth=1, markersize=2)
        plt.savefig('trajetory.png', dpi=400)
        plt.show()
#print(path)


        
    def to_pixel(self, GPS_point, zoom=19):
        """
            Generates an X,Y pixel coordinate based on the latitude, longitude
            and zoom level
            Returns:    An X,Y pixel coordinate
        """
        #check if we are inside the map
        if (GPS_point[0]>self.NW_corner[0]) or (GPS_point[1]<self.NW_corner[1]) or (GPS_point[0]<self.SE_corner[0]) or (GPS_point[1]>self.SE_corner[1]):
            raise CustomError("point is outside the map")

        tile_size = 256 #each tile has 256 pixels

        # Use a left shift to get the power of 2, zoom 0 is a world map, zoom 1 is wold map divided in 4 tiles, zoom 2 is world map divided in 16 tiles
        # i.e. a zoom level of 2 will have 2^2 = 4 tiles as coordinate sistem divides axis in 4 (x+y+,x-y+,x-y-,x+y-)
        numTiles = 1 << zoom

        # Find the x_pixel given the longitude
        x_pixel = (tile_size / 2 + GPS_point[1] * tile_size / 360.0) * numTiles 

        # Convert the latitude to radians and take the sine
        sin_y = math.sin(GPS_point[0] * (math.pi / 180.0))

        # Calulate the y_pixel
        y_pixel = ((tile_size / 2) + 0.5 * math.log((1 + sin_y) / (1 - sin_y)) * -(
        tile_size / (2 * math.pi))) * numTiles 
        
        return [x_pixel-self.corner_pixel[0], y_pixel-self.corner_pixel[1]]
    
    def calculate_corner_pixel(self):
        """
            Generates an X,Y pixel coordinate based on the latitude, longitude
            and zoom level
            Returns:    An X,Y pixel coordinate
        """

        tile_size = 256 #each tile has 256 pixels

        # Use a left shift to get the power of 2, zoom 0 is a world map, zoom 1 is wold map divided in 4 tiles, zoom 2 is world map divided in 16 tiles
        # i.e. a zoom level of 2 will have 2^2 = 4 tiles as coordinate sistem divides axis in 4 (x+y+,x-y+,x-y-,x+y-)
        numTiles = 1 << 19

        # Find the x_pixel given the longitude
        x_pixel = (tile_size / 2 + self.NW_corner[1] * tile_size / 360.0) * numTiles 

        # Convert the latitude to radians and take the sine
        sin_y = math.sin(self.NW_corner[0] * (math.pi / 180.0))

        # Calulate the y_pixel
        y_pixel = ((tile_size / 2) + 0.5 * math.log((1 + sin_y) / (1 - sin_y)) * -(
        tile_size / (2 * math.pi))) * numTiles 
        
        self.corner_pixel=[x_pixel, y_pixel]
            
        
def main():
        

    #read map
    mapp= Plan("Loyola.png","Loyolaplantilla.png","plantilla.csv", "grid.csv")
    start=[37.30756124092905, -5.940235027872745]
    destination=[37.308586181428446, -5.939682210759604]
    mapp.goto(start, destination)
    


if __name__ == '__main__':  main()