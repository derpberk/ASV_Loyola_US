import threading

import paho.mqtt.client as mqtt


class MQTT(object):
    def __init__(self, name, addr='127.0.0.1', port=1883, timeout=60, topics2suscribe=None, on_message=None):

        self.client = mqtt.Client(name)

        if topics2suscribe is None:
            topics2suscribe = []
        self.topics2suscribe = topics2suscribe

        if on_message is None:
            self.onmessage = self.on_message
        else:
            self.onmessage = on_message
        # Con esto se puede mejorar la seguridad del MQTT si el broker esta configurado para eso
        # self.client.username_pw_set("", "")

        self._mqtt_thread = threading.Thread(target=self.mqtt_thread, args=(addr, port, timeout,))
        self._mqtt_thread.start()

    def mqtt_thread(self, addr='127.0.0.1', port=1883, timeout=60):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.onmessage
        self.client.connect(addr, port, timeout)
        self.client.loop_forever()

    def on_connect(self, _client, _, __, rc):
        print("Connected to MQTT server")
        for topic in self.topics2suscribe:
            self.client.subscribe(topic)

    @staticmethod
    def on_message(_client, user_data, msg):
        message = bool(msg.payload)
        print(message)

    def send_new_msg(self, msg, topic="coordinator"):
        while not self.client.is_connected():
            continue
        self.client.publish(topic, msg)
