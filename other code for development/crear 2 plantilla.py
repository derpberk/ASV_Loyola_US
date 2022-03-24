
# modified from the scripts created by Hayden Eskriett [http://eskriett.com] and jbndlr
# Find the associated blog post at: http://blog.eskriett.com/2013/07/19/downloading-google-maps/
# but it has been done in a better way using the data in https://developers.google.com/maps/documentation/javascript/coordinates
# and https://stackoverflow.com/questions/40342355/how-can-i-generate-a-regular-geographic-grid-using-python





###################################IMPORTANT##################################
#Google maps works with [Latitude Longitude] variables however
#Dronekit and other programs (such as gridder) work with [Longitude Latitude] variables
##############################################################################

from PIL import Image
import numpy as np

class CustomError(Exception):
     pass

class mapper:
    
    def __init__(self, grid, image, plantilla, plantilla2):
        """
            Args:
                grid:       The [Lat Lon] points of the grid as well as the size, generated in create grid.py
                image:      image printed in create grid
                plantilla:  black and white image photoshoped from image (black=wall) (white=path)
        """
        
        
        
        self.read_from_file(grid)
        self.img=Image.open(plantilla) #open mask
        self.img2=Image.open(plantilla2) #open mask
        self.grayImage=self.img.convert(mode="L") #convert to grey scale
        self.grayImage2=self.img2.convert(mode="L") #convert to grey scale
        #self.img.show()
        
        #TODO: change zoom and check grid size, if it is different than 1 you should say whats obstacle and what is path
            # useful to reduce computation time
        
        img=Image.open(image) #open original image
        image1 = self.grayImage.convert("RGBA") #convert both images to RGBA
        image2 = img.convert("RGBA")
        image3 = self.grayImage2.convert("RGBA") #convert both images to RGBA
        
        alphaBlended1 = Image.blend(image3, image1, alpha=.6) #blend images
        alphaBlended1 = Image.blend(alphaBlended1, image2, alpha=.4) #blend images
            
        alphaBlended1.show() #show to check mask
        
        
    def save_to_file(self, file):
        
        array = np.array(self.grayImage) #convert to array
        np.savetxt(file, array<128, fmt="%d") #save to file as 1 wall, 0 path
        
    def save2_to_file(self, file):
        
        array = np.array(self.grayImage2) #convert to array
        np.savetxt(file, array<128, fmt="%d") #save to file as 1 wall, 0 path
    
    
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
            
        
def main():
        

    #read map
    mapp= mapper("grid.csv","Loyola.png","Loyolaplantilla.png", "Loyolaplantilla2.png")
    mapp.save_to_file("plantilla.csv")
    mapp.save2_to_file("plantilla2.csv")

if __name__ == '__main__':  main()