import numpy as np
import xml.etree.ElementTree as ElementTree
from asv_interfaces.msg import Location


class KMLMissionGenerator:

    def __init__(self, mission_directory):

        self.root = root = ElementTree.parse(mission_directory).getroot()
        self.root_head = self.root.tag[:-3]

        self.missions = []

        missions_list = list(root[0].findall(self.root_head + 'Placemark'))

        self.num_of_missions = len(missions_list)

        for mission in missions_list:
            coord = mission.findall(self.root_head + 'LineString')[0]
            coord = coord.findall(self.root_head + 'coordinates')[0]
            coord_str = coord.text
            coord_lst = coord_str.split(' ')[:-1]

            mission_array = []
            for point in coord_lst:
                coord_point = [float(elem) for elem in point.split(',')]

                mission_array.append(coord_point)

            self.missions.append(mission_array)

    def get_mission_list(self):

        return self.missions

    def get_samplepoints(self):
        samplepoints=[]
        for point in self.missions[0]:
            p = Location()
            p.lat=point[1]
            p.lon=point[0]
            #p.alt=point[2]
            samplepoints.append(p)
        return samplepoints

    def plot_mission(self, mission_nu=0):

        import matplotlib.pyplot as plt

        mission = np.array(self.missions[mission_nu])
        fig, ax = plt.subplots(1, 1)
        ax.plot(mission[:, 0], mission[:, 1], 'r-o')
        ax.ticklabel_format(axis='both', style='sci')

        for n, elem in enumerate(mission):
            plt.text(elem[0], elem[1], str(n), style='italic')
        ax.grid()

        ax.set_xlabel('LATITUD')
        ax.set_ylabel('LONGITUD')
        ax.set_title(f'Mision Nº {mission_nu}')
        plt.show()


if __name__ == '__main__':
    mg = KMLMissionGenerator('kml_files/Misiones_Dron_US_Loyola_2.kml')
    li = mg.get_mission_list()

    for m in range(mg.num_of_missions):
        print(np.array(li[m]))
        mg.plot_mission(m)
