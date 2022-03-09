import threading
import paho.mqtt.client as mqtt
import subprocess  # For executing a shell command
import os
import signal
import psutil
def ping_google():
    host="www.google.es"
    command = ["ping", "-c", "1", "-W2", host] #timeout 2 seconds

    return subprocess.run(args=command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 #this returns 0 if ping is successfull
def ping_mqtt():
    host="127.0.0.1"
    command = ["ping", "-c", "1", "-W2", host] #timeout 2 seconds

    return subprocess.run(args=command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 #this returns 0 if ping is successfull
    
def check_ssh_tunelling():

    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if "ssh" == pinfo['name'].lower():
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False;

def kill_ssh_tunelling():
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            pinfo = proc.as_dict(attrs=['pid', 'name', 'create_time'])
            if "ssh" == pinfo['name'].lower():
                proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

def start_ssh_tunneling():
    #arg i to understand aliases
    #stdout and stderr to do not print bash messages
    #preexec_fn so that we can close the command executed in bash and it doesnt keep running
    if subprocess.run(args=['/bin/bash', '-i', '-c', "mqttssh"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid).returncode == 0:
        return True
    return False

    #TODO: observar que valores se devuelven cuando hay error de ssh, se desconoce si lo que devuelve cuando ssh falla

class MQTT(object):
    def __init__(self, name, addr='127.0.0.1', port=1883, timeout=60, topics2suscribe=None, on_message=None, on_disconnect=None):

        self.client = mqtt.Client(name)

        if topics2suscribe is None:
            topics2suscribe = []
        self.topics2suscribe = topics2suscribe

        if on_message is None:
            self.onmessage = self.on_message
        else:
            self.onmessage = on_message

        if on_disconnect is None:
            self.on_disconnect = self.on_disconnect
        else:
            self.on_disconnect = on_disconnect
        # Con esto se puede mejorar la seguridad del MQTT si el broker esta configurado para eso
        # self.client.username_pw_set("", "")

        self._mqtt_thread = threading.Thread(target=self.mqtt_thread, args=(addr, port, timeout,))
        self._mqtt_thread.start()

    def mqtt_thread(self, addr='127.0.0.1', port=1883, timeout=60):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.onmessage
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(addr, port, timeout)
        self.client.loop_forever()

    def on_connect(self, _client, _, __, rc):
        print("Connected to MQTT server")
        for topic in self.topics2suscribe:
            self.client.subscribe(topic)

    def on_disconnect(client, userdata,_, rc):
        print("client disconnected ok")

    @staticmethod
    def on_message(_client, user_data, msg):
        message = bool(msg.payload)
        print(message)

    def send_new_msg(self, msg, topic="coordinator"):
        #while not self.client.is_connected():
            #continue
        self.client.publish(topic, msg)
