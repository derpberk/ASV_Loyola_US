import os
from time import sleep
from ArduinoSerialBoard import SerialBoard
import RPi.GPIO as GPIO

class WaterPumpModule():

    def __init__(self, serial_string="/dev/ttyACM0",  charging_time=5, discharging_time=5, mode='BuiltInPin'):

        assert charging_time > 0 and discharging_time > 0, "CHARGE/DISCHARGE TIME MUST BE GREATER THAN 0!"
        assert mode in ['SerialBoard', 'BuiltInPin'], "SerialBoard/BuiltInPin are the only permitted modes!"

        self.mode = mode

        if mode == 'BuiltInPin':
            self.activation_pin = 26
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.activation_pin, GPIO.LOW)
        else:
            self.serial_board = SerialBoard(ser_path=serial_string, baudrate=9600, timeout=None)

        self.charging_time = charging_time
        self.discharging_time = discharging_time

        self.set_pump_relay_value('LOW')

    def set_pump_relay_value(self, value):

        assert value in ['HIGH', 'LOW'], "THE ONLY ACCEPTED VALUES FOR set_pump_relay_value are HIGH/LOW"

        if self.mode == 'SerialBoard':
            self.serial_board.write_gpio(value)
        else:
            val = GPIO.LOW if value == 'LOW' else GPIO.HIGH
            GPIO.output(self.activation_pin,val)

    def charge_probe(self):

        # Move water for a time #
        self.set_pump_relay_value('HIGH')
        # Wait until the probe is full #
        sleep(self.charging_time)
        # When full, deactivate the pump #
        self.set_pump_relay_value('LOW')

    def discharge_probe(self):

        # Activate the pump to discharge
        self.set_pump_relay_value('HIGH')
        # Wait for the discharge time
        sleep(self.discharging_time)
        # Deactivate the pump
        self.set_pump_relay_value('LOW')















