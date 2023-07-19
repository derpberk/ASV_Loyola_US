
# modified from the scripts created by Hayden Eskriett [http://eskriett.com] and jbndlr
# Find the associated blog post at: http://blog.eskriett.com/2013/07/19/downloading-google-maps/
# but it has been done in a better way using the data in https://developers.google.com/maps/documentation/javascript/coordinates
# and https://stackoverflow.com/questions/40342355/how-can-i-generate-a-regular-geographic-grid-using-python





###################################IMPORTANT##################################
#Google maps works with [Latitude Longitude] variables however
#Dronekit and other programs (such as gridder) work with [Longitude Latitude] variables
##############################################################################

import urllib.request
from PIL import Image
import os
import math
import pyproj
import matplotlib.pyplot as plt
import numpy as np
from KMLMissionGeneration import KMLMissionGenerator
class CustomError(Exception):
     pass

class GoogleMapDownloader:
    """
        A class which generates high resolution google maps image given
        Northwest and Southeast points in [Lat Lon] and zoom level
        You can change the quality of the image by changing the zoom and layer values
        
        19 is a valid zoom value knowing the size of our drone
        
        for the maps you have available
        ROADMAP = "v"
        TERRAIN = "p"
        ALTERED_ROADMAP = "r"
        SATELLITE = "s"
        TERRAIN_ONLY = "t"
        HYBRID = "y"
        
        
    """

    def __init__(self, Northwest, Southeast, zoom=19, layer="s"):
        """
            GoogleMapDownloader Constructor
            Args:
                lat:    The latitude of the location required
                lng:    The longitude of the location required
                zoom:   The zoom level of the location required, ranges from 0 - 23
        """
        self.Northwest = Northwest
        self.Southeast = Southeast
        self._zoom = zoom
        self._layer = layer
        
        if Northwest[0]<=Southeast[0] or Northwest[1]>=Southeast[1]:
            raise CustomError("You must use Northwest(top-left) and Southeast(bottom-right) coordinates")
        self.get_nw_coordinates()
        self.get_se_coordinates()
        
        if (abs(self.nw_tile[0]-self.se_tile[0]) == 0) or (abs(self.nw_tile[1]-self.se_tile[1]) == 0):
            raise CustomError("Insuficient zoom, points are too close, i am lazy to program this, increase zoom")
        
        

    def get_nw_coordinates(self):
        """
            Generates an X,Y tile and pixel coordinate based on the latitude, longitude
            and zoom level
            Returns:    An X,Y pixel coordinate
        """

        tile_size = 256 #each tile has 256 pixels

        # Use a left shift to get the power of 2, zoom 0 is a world map, zoom 1 is wold map divided in 4 tiles, zoom 2 is world map divided in 16 tiles
        # i.e. a zoom level of 2 will have 2^2 = 4 tiles as coordinate sistem divides axis in 4 (x+y+,x-y+,x-y-,x+y-)
        numTiles = 1 << self._zoom

        # Find the x_pixel given the longitude
        x_pixel = (tile_size / 2 + self.Northwest[1] * tile_size / 360.0) * numTiles 

        # Convert the latitude to radians and take the sine
        sin_y = math.sin(self.Northwest[0] * (math.pi / 180.0))

        # Calulate the y_pixel
        y_pixel = ((tile_size / 2) + 0.5 * math.log((1 + sin_y) / (1 - sin_y)) * -(
        tile_size / (2 * math.pi))) * numTiles 
        
        self.nw_pixel=[x_pixel, y_pixel]
        self.nw_tile=[int(x_pixel // tile_size), int(y_pixel // tile_size)]
    
    def get_se_coordinates(self):
        """
            Generates an X,Y tile and pixel coordinate based on the latitude, longitude
            and zoom level
            Returns:    An X,Y pixel coordinate
        """

        tile_size = 256 #each tile has 256 pixels

        # Use a left shift to get the power of 2, zoom 0 is a world map, zoom 1 is wold map divided in 4 tiles, zoom 2 is world map divided in 16 tiles
        # i.e. a zoom level of 2 will have 2^2 = 4 tiles as coordinate sistem divides axis in 4 (x+y+,x-y+,x-y-,x+y-)
        numTiles = 1 << self._zoom

        # Find the x_pixel given the longitude
        x_pixel = (tile_size / 2 + self.Southeast[1] * tile_size / 360.0) * numTiles 

        # Convert the latitude to radians and take the sine
        sin_y = math.sin(self.Southeast[0] * (math.pi / 180.0))

        # Calulate the y_pixel
        y_pixel = ((tile_size / 2) + 0.5 * math.log((1 + sin_y) / (1 - sin_y)) * -(
        tile_size / (2 * math.pi))) * numTiles 
        
        self.se_pixel=[x_pixel, y_pixel]
        self.se_tile=[math.ceil(x_pixel // tile_size)+1, math.ceil(y_pixel // tile_size)+1]
            
    def generateImage(self, ):
        """
            Generates an image by stitching a number of google map tiles together.

            Args:
                start_x:        The top-left x-tile coordinate
                start_y:        The top-left y-tile coordinate
                tile_width:     The number of tiles wide the image should be -
                                defaults to 5
                tile_height:    The number of tiles high the image should be -
                                defaults to 5
            Returns:
                A high-resolution Goole Map image.
        """
        tile_width = abs(self.nw_tile[0]-self.se_tile[0])
        tile_height = abs(self.nw_tile[1]-self.se_tile[1])
        
        
            
        # Determine the size of the image, for simplicity, we will print the whole image and later crop it
        width, height = 256 * tile_width, 256 * tile_height

        # Create a new image of the size required
        map_img = Image.new('RGB', (width, height))
        
        
        # calculate northwest pixel of the tile
        start_x=self.nw_tile[0]
        start_y=self.nw_tile[1]

        # print the map
        for x in range(0, tile_width):
            for y in range(0, tile_height):
                
                url = f'https://mt0.google.com/vt?lyrs={self._layer}&x=' + str(start_x + x) + '&y=' + str(start_y + y) + '&z=' + str(
                    self._zoom)

                current_tile = str(x) + '-' + str(y)
                urllib.request.urlretrieve(url, current_tile)

                im = Image.open(current_tile)
                map_img.paste(im, (x * 256, y * 256))

                os.remove(current_tile)
        
        
        # cut corners
        left=abs(self.nw_tile[0] * 256 - self.nw_pixel[0])
        top=abs(self.nw_tile[1] * 256 - self.nw_pixel[1])
        right=left + abs(self.se_pixel[0] - self.nw_pixel[0])
        bottom=top + abs(self.nw_pixel[1] - self.se_pixel[1])
        
        
        map_img = map_img.crop((left, top, right, bottom))
        return map_img

class gridder:
    
    def __init__(self, NW_corner, SE_corner, grid_size):
        """
            Args:
                NW_corner:
                SE_corner:   
                grid_size:   
        """
        
        
        
        # Set up transformers, EPSG:3857 is metric, same as EPSG:900913
        self.to_proxy_transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:3857')
        self.to_original_transformer = pyproj.Transformer.from_crs('epsg:3857', 'epsg:4326')
        
        # Project corners to target projection
        
        #this transformation works in [Longitude Latitude]
        self.transformed_nw = self.to_proxy_transformer.transform(NW_corner[1], NW_corner[0]) # Transform NW point to 3857
        self.transformed_se = self.to_proxy_transformer.transform(SE_corner[1], SE_corner[0]) # .. same for SE

        self.NW_corner=NW_corner
        self.SE_corner=SE_corner
        self.grid_size=grid_size
        print(f"map size is {int(abs(self.transformed_nw[1]-self.transformed_se[1]))}x{int(abs(self.transformed_nw[0]-self.transformed_se[0]))}m")
        self.create_grid()
        print(f"grid size is {self.width}x{self.heigth} cells")
        self.calculate_pixels_per_grid()
        
        
        
        
        
    def save_to_file(self, name):
        with open(name, 'w') as of:
            for j in range(self.heigth):
                for i in range(self.width):
                    of.write('{:f},{:f};'.format(self.gridpoints[j][i][0], self.gridpoints[j][i][1]))
                of.write('\n')
            
            
    def create_grid(self):
        # Iterate over 2D area
        self.gridpoints = []
        countx=0
        county=0
        x = self.transformed_nw[0]
        while x > self.transformed_se[0]:
            county+=1
            countx=0
            column=[]
            y = self.transformed_nw[1]
            while y < self.transformed_se[1]:
                p = self.to_original_transformer.transform(x, y)
                column.append([p[1],p[0]])
                y += self.grid_size
                countx+=1
            column.pop() #cut corners
            self.gridpoints.append(column)
            x -=self.grid_size
        self.gridpoints.pop() #cut corners
        self.width=len(self.gridpoints[0])
        self.heigth=len(self.gridpoints)
            
        
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
    
    def calculate_pixels_per_grid(self):
        
        self.calculate_corner_pixel()
        p=self.to_pixel(self.gridpoints[self.heigth-1][self.width-1])
        x=p[0]
        y=p[1]
        self.pixel_width=x/(self.width-1)
        self.pixel_heigth=y/(self.heigth-1)
        print(x,y)
            
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
        
    def show_grid(self, img):
            
        #show grid
        x=[]
        y=[]
        plt.figure()
        plt.imshow(img, cmap=plt.get_cmap('binary'))
        
        #horizontal lines
        for j in range(self.heigth):
            for i in range(self.width):
                p=self.to_pixel(self.gridpoints[j][i])
                x.append(p[0])
                y.append(p[1])
            plt.plot(x,y,'b-', linewidth=0.2)
            x=[]
            y=[]
        #vertical lines
        
        for i in range(self.width):
            for j in range(self.heigth):
                p=self.to_pixel(self.gridpoints[j][i])
                x.append(p[0])
                y.append(p[1])
            plt.plot(x,y,'b-', linewidth=0.2)
            x=[]
            y=[]
        plt.savefig('grid.png', dpi=400)
        
        img=Image.open('grid.png')
        img.show()
        
        
    def show_point(self, img, point):
        
        #print a point in the map
        pixel=self.to_pixel(point)
        
        plt.figure()
        plt.imshow(img, cmap=plt.get_cmap('binary'))
        plt.plot(pixel[0],pixel[1],'ro', markersize=5)
        plt.show()
        
    def save_empty_mask(self, file):
        with open(file, 'w') as of:
            for j in range(self.heigth-1):
                for i in range(self.width-1):
                    of.write('0;')
                of.write('\n')
                
    def save_mask(self, file):
        with open(file, 'w') as of:
            for j in range(self.heigth-1):
                for i in range(self.width-2):
                    of.write('{:d};'.format(self.mask[j][i]))
                of.write('{:d}\n'.format(self.mask[j][i]))
        
    def create_mask(self, img):
        grayImage=img.convert(mode="L")
        array = np.array(grayImage)
        self.mask=[]
        for j in range(self.heigth-1):
            column=[]
            for i in range(self.width-1):
                p1=self.to_pixel(self.gridpoints[j][i])
                p2=self.to_pixel(self.gridpoints[j+1][i])
                p3=self.to_pixel(self.gridpoints[j][i+1])
                p4=self.to_pixel(self.gridpoints[j+1][i+1])
                p1=array[int(p1[1])][int(p1[0])]
                p2=array[int(p2[1])][int(p2[0])]
                p3=array[int(p3[1])][int(p3[0])]
                p4=array[int(p4[1])][int(p4[0])]
                white=0
                if p1:
                    white+=1
                if p2:
                    white+=1
                if p3:
                    white+=1
                if p4:
                    white+=1
                if white>2:
                    column.append(0)
                else:
                    column.append(1)
            self.mask.append(column)
    def show_mask(self, img):
        #show grid
        
        plt.figure()
        plt.imshow(img, cmap=plt.get_cmap('binary'))
        
        for j in range(self.heigth-1):
            for i in range(self.width-1):
                if not self.mask[j][i]:
                    x=[]
                    y=[]
                    p1=self.to_pixel(self.gridpoints[j][i])
                    p2=self.to_pixel(self.gridpoints[j+1][i])
                    p3=self.to_pixel(self.gridpoints[j][i+1])
                    p4=self.to_pixel(self.gridpoints[j+1][i+1])
                    x.append(p1[0])
                    y.append(p1[1])
                    x.append(p2[0])
                    y.append(p2[1])
                    x.append(p4[0])
                    y.append(p4[1])
                    x.append(p3[0])
                    y.append(p3[1])
                    x.append(p1[0])
                    y.append(p1[1])
                    plt.plot(x,y,'b-', linewidth=0.2)
                    
        
        plt.savefig('mask.png', dpi=400)
        
        img=Image.open('mask.png')
        img.show()
        
    def show_mission(self, img):
        plt.figure()
        plt.imshow(img, cmap=plt.get_cmap('binary'))
        point=0
        KML=KMLMissionGenerator("MisionesLoyola_dron_3.kml")
        mission=KML.get_mission_list()[0]
        
        for j in range(self.heigth-1):
            for i in range(self.width-1):
                if not self.mask[j][i]:
                    x=[]
                    y=[]
                    p1=self.to_pixel(self.gridpoints[j][i])
                    p2=self.to_pixel(self.gridpoints[j+1][i])
                    p3=self.to_pixel(self.gridpoints[j][i+1])
                    p4=self.to_pixel(self.gridpoints[j+1][i+1])
                    x.append(p1[0])
                    y.append(p1[1])
                    x.append(p2[0])
                    y.append(p2[1])
                    x.append(p4[0])
                    y.append(p4[1])
                    x.append(p3[0])
                    y.append(p3[1])
                    x.append(p1[0])
                    y.append(p1[1])
                    plt.plot(x,y,'b-', linewidth=0.2)
        for i in mission:
            point=[i[1], i[0]]
            pixel=self.to_pixel(point)
            plt.plot(pixel[0],pixel[1],'ro', markersize=0.5)
        
        
        plt.savefig('mission.png', dpi=400)
        
        img=Image.open('mission.png')
        img.show()
                    
                    
def main():
    # Create a new instance of GoogleMap Downloader
    """
    Loyola
    NW_corner=[37.30877610927852, -5.940516378146266] #northwest point of the map [latitude longitude]
    SE_corner=[37.30661986681041, -5.939422309058239] #southeast point of the map [latitude longitude]
    
    NW_corner=[37.420088017565696, -6.001345595400041] #northwest point of the map [latitude longitude]
    SE_corner=[37.41837443521307, -5.997484063063451] #southeast point of the map [latitude longitude]
    #alamillo
    """
    
    NW_corner=[37.19141337236496, -4.39886029163363] #northwest point of the map [latitude longitude]
    SE_corner=[37.18828908755112, -4.39441716422766] #southeast point of the map [latitude longitude]
    #alamillo
    
    #SE_corner=[37.29989089621092, -5.9274044593699236]
    grid_size= 2     #size of the grid in meters
    
    #create class of the map downloader 
    gmd = GoogleMapDownloader(NW_corner ,SE_corner) #zoom 19, aproximadamente tiles de 50m^2, más zoom no se aprecian obstaculos nuevos
    
    try:
        # Get the high resolution image
        img = gmd.generateImage()
    except IOError:
        print("Could not generate the image - try adjusting the zoom level and checking your coordinates")
    else:
        # Save the image to disk
        img.rotate(17)
        img.save("high_resolution_image2.png")
        print("The map has successfully been created")
        
    
    #Show the image
    plt.figure()
    plt.imshow(img, cmap=plt.get_cmap('binary'))
    plt.show()
    
    #create grid
    grid= gridder(NW_corner,SE_corner,grid_size)


    #show the grid on the image
    aux=0
    while aux!="y" and aux!="n":
        aux=input("is there already an existing mask (y/n)").lower()
    if aux=='n':
        grid.show_grid(img)
        grid.save_to_file("grid.csv")
        grid.save_empty_mask("empty_mask.csv")
        return
    
    while not os.path.exists(str("./"+aux)) or aux[-1]!="g":
        aux=input("insert mask name").lower()
    imgn=Image.open(aux)
    grid.show_grid(imgn)
    name=aux[0:-4]
    grid.create_mask(imgn)
    grid.save_mask(str(name+".csv"))
    grid.show_mask(img)


    #show a point
    #grid.show_point(img, [37.307659405593995, -5.940044140455814])
    
    #grid.show_mission(img)
        


if __name__ == '__main__':  main()