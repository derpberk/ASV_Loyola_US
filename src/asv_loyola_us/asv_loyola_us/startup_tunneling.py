from time import sleep
from submodulos.MQTT import MQTT
from submodulos.terminal_handler import ping_google, check_ssh_tunelling, start_ssh_tunneling, kill_ssh_tunelling, kill_ros2
import json, traceback
from datetime import datetime
import os
import subprocess  # For executing a shell command

vehicle_id=2
class startup:
    def __init__(self):
        kill_ros2() #if this program is called from a crash, close last ros2 session
        self.asv_offline=True #we start offline
        kill_ssh_tunelling()
        while not ping_google(): #ping google has an internal delay
            print("There is no internet connection, retrying...")
        try:
            if not check_ssh_tunelling(): #ssh tunelling is not running
                if not start_ssh_tunneling(): #ssh tunelling could not be made
                    print("ssh tunneling failed, there is no connection to server")
                else:
                    print("ssg tunneling started")
            else:
                print("ssg tunneling was running")
        except:
            error = traceback.format_exc()
            print(f"Tunelling connection failed, unknown error:\n {error}")

        try:
            print(f"MQTT connecting to 127.0.0.1")
            self.mqtt = MQTT(str(vehicle_id), addr="127.0.0.1", topics2suscribe=[f"veh{vehicle_id}"], on_message=self.on_message, on_disconnect=self.on_disconnect)
        except ConnectionRefusedError:
            print(f"Connection to MQTT server was refused")
        except OSError:
            print(f"MQTT server was not found")
        except TimeoutError:
            print(f"MQTT was busy, timeout error")
        except:
            error = traceback.format_exc()
            print(f"MQTT connection failed, unknown error:\n {error}")

        sleep(2) #wait for connection
        message = json.dumps({
            "veh_num": vehicle_id,
            "origin node": "startup",
            "time": str(datetime.now()),
            "msg": "Drone turned on"
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="log")  # Send the MQTT message
        while True:
            while self.asv_offline:
                msg = json.dumps({
                    "veh_num": vehicle_id,
                    "armed": False,
                    "mission_mode": "Waiting for server",
                    "asv_mode": "OFFLINE"
                })  # Must be a JSON format file.
                #TODO: transformar el topic con la informacion a formato JSON
                self.mqtt.send_new_msg(msg)  # Send the MQTT message
            sleep(1)


    def on_message(self, _client, _, msg):
        if msg.topic == f"veh{vehicle_id}": 
            message = json.loads(msg.payload.decode('utf-8'))  # Decode the msg into UTF-8
            print(f"Received {message} on topic {msg.topic}")
            self.asv_offline=False
            self.mqtt.close()
            subprocess.run(args=['/bin/bash', '-i', '-c', "roslaunch"], preexec_fn=os.setsid)

    def on_disconnect(self, _client, __, _):
        print("connection to server was lost")




if __name__ == '__main__':
    S=startup()