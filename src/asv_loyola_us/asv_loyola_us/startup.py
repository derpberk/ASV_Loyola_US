from time import sleep
from submodulos.MQTT import MQTT
from submodulos.terminal_handler import ping_google, check_ssh_tunelling, start_ssh_tunneling, kill_ssh_tunelling, kill_ros2, restart_asv, check_internet
import json, traceback
from datetime import datetime
import os
import subprocess  # For executing a shell command

def get_vehicle_id():
    with open('/home/xavier/ASV_Loyola_US/src/asv_loyola_us/config/config.yaml', 'r') as f:
        for line in f:
            line=line.strip().replace(" ", "").split(":")
            if line[0]=="vehicle_id":
                return int(line[1])

class startup:
    def __init__(self):
        self.vehicle_id=get_vehicle_id()
        kill_ros2() #if this program is called from a crash, close last ros2 session
        self.asv_offline=True #we start offline
        kill_ssh_tunelling()
        while not ping_google():
            print("There is no internet connection, retrying...")
            sleep(1)
        try:
            print(f"MQTT connecting to dronesloyolaus.eastus.cloudapp.azure.com")
            self.mqtt = MQTT(str(self.vehicle_id), addr="dronesloyolaus.eastus.cloudapp.azure.com", topics2suscribe=[f"veh{self.vehicle_id}"], on_message=self.on_message, on_disconnect=self.on_disconnect)
        except ConnectionRefusedError:
            print(f"Connection to MQTT server was refused")
        except OSError:
            print(f"MQTT server was not found")
        except TimeoutError:
            print(f"MQTT was busy, timeout error")
        except:
            error = traceback.format_exc()
            print(f"MQTT connection failed, unknown error:\n {error}")

        print(f"Drone {self.vehicle_id} started")

        sleep(2) #wait for connection
        name="Drone "+str(self.vehicle_id)+" turned on"
        message = json.dumps({
            "veh_num": self.vehicle_id,
            "origin node": "startup",
            "time": str(datetime.now()),
            "msg": str(name)
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="log")  # Send the MQTT message
        while self.asv_offline:
            msg = json.dumps({
                "veh_num": self.vehicle_id,
                "armed": False,
                "mission_mode": "Waiting for server",
                "asv_mode": "OFFLINE"
            })  # Must be a JSON format file.
            #TODO: transformar el topic con la informacion a formato JSON
            self.mqtt.send_new_msg(msg)  # Send the MQTT message
            sleep(1)
            if not ping_google():
                #we lost connection, restart
                print("Connection closed, restarting program")
                restart_asv()
                exit()



        msg = json.dumps({
            "veh_num": self.vehicle_id,
            "armed": False,
            "mission_mode": "Starting ROS2",
            "asv_mode": "OFFLINE"
        })  # Must be a JSON format file.
        name="Drone "+str(self.vehicle_id)+" starting ROS2"
        message = json.dumps({
            "veh_num": self.vehicle_id,
            "origin node": "startup",
            "time": str(datetime.now()),
            "msg": str(name)
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="log")  # Send the MQTT message
        sleep(0.1)
        #TODO: transformar el topic con la informacion a formato JSON
        self.mqtt.send_new_msg(msg)  # Send the MQTT message
        self.mqtt.close()
        subprocess.run(args=['/bin/bash', '-i', '-c', "roslaunch"], preexec_fn=os.setsid)


    def on_message(self, _client, _, msg):
        if msg.topic == f"veh{self.vehicle_id}": 
            message = json.loads(msg.payload.decode('utf-8'))  # Decode the msg into UTF-8
            print(f"Received {message} on topic {msg.topic}")
            self.asv_offline=False

    def on_disconnect(self, _client, __, _):
        print("connection to server was lost")




if __name__ == '__main__':
    S=startup()