import serial
import time
import sqlite3
from datetime import datetime
from PumpModule import WaterPumpModule
import copy

sensor_keys = ['SAMPLE_NUM', 'BAT', 'WT', 'PH', 'DO', 'LATITUD', 'LONGITUD', 'COND', 'ORP', 'DATE']


class WaterQualityModule():
	""" Class for the water quality module sensor aka Libellium. """

	def __init__(self, database_name='LOCAL_DATABASE.db', USB_string='USBPort1', timeout=6, baudrate=115200,
	             pump_parameters=None):

		""" Create the serial object """
		self.serial = serial.Serial('/dev/tty' + USB_string, baudrate, timeout=timeout)

		""" Connect to the Database """
		self.database = sqlite3.connect(database_name)

		""" Create the cursor to manage the database"""
		self.cursor = self.database.cursor()

		""" Initialize the data dictionary of the sensor measurements """
		self.sensor_data = {}
		for key in sensor_keys:
			self.sensor_data[key] = -1  # A -1 value indicates a faulty value #

		""" Create the Pump Module object """
		if pump_parameters is None:
			pump_parameters = {'charging_time': 6,
			                   'discharging_time': 1.2,
			                   'serial_string': '/dev/ttyACM0',
			                   'mode': 'BuiltInPin'}

		self.pump = WaterPumpModule(charging_time=pump_parameters['charging_time'],
		                            discharging_time=pump_parameters['discharging_time'],
		                            mode=pump_parameters['mode'],
		                            serial_string=pump_parameters['serial_string'])

	def take_a_sample(self, position, num_of_samples=1, drone_id=-1):
		""" Take num_of_samples and save the data with the given position in the database """

		""" Charge the pump!"""
		print("Charging the pump")
		self.pump.charge_probe()

		# list of dicts that go into db
		sample_strs = []

		# Iterate over the sample_nums
		for i in range(num_of_samples):
			print("Taking sample {} / {}".format(i + 1, num_of_samples))

			self.read_frame()  # Read a frame from the buffer

			str_date = str(datetime.now())  # Leemos la fecha y la convertimos en string
			# Metemos la fecha en el diccionario de variables
			self.sensor_data['DATE'] = str_date

			# Almacenamos la posicion
			self.sensor_data['LATITUD'] = position[0]
			self.sensor_data['LONGITUD'] = position[1]

			print("Incoming data: ")
			print(self.sensor_data)

			# creamos una tupla de parametros que nos permitira introducir los datos en la tabla sensor
			parametros = (drone_id,
			              self.sensor_data['SAMPLE_NUM'],
			              self.sensor_data['BAT'],
			              self.sensor_data['WT'],
			              self.sensor_data['PH'],
			              self.sensor_data['DO'],
			              self.sensor_data['LATITUD'],
			              self.sensor_data['LONGITUD'],
			              self.sensor_data['COND'],
			              self.sensor_data['ORP'],
			              self.sensor_data['DATE'])

			# insertamos valores en nuestra tabla "sensor"
			self.cursor.execute("INSERT INTO sensor (ID,SAMPLE_NUM,BAT,TEMP,PH,DO,LATITUD,LONGITUD,COND,ORP,DATE) VALUES(?,?,?,?,?,?,?,?,?,?,?)",parametros)

			# el siguiente comando "commit()" guarda la tabla creada
			self.database.commit()

			# Append del diccionario #
			sample_strs.append(copy.copy(self.sensor_data))

		""" Discharge the pump!"""
		print("Discharging the pump!")
		self.pump.discharge_probe()

		return sample_strs  # Return the last readed values in a list of dictionaries #

	def read_frame(self):

		is_frame_ok = False  # While a frame hasnt correctly readed #
		self.serial.reset_input_buffer()  # Erase the input buffer to start listening

		while not is_frame_ok:

			time.sleep(0.5)  # Polling time. Every 0.5 secs, check the buffer #

			if self.serial.inWaiting() < 27:  # If the frame has a lenght inferior to the minimum of the Header
				continue

			else:

				try:
					bytes = self.serial.read_all()  # Read all the buffer #

					bytes = bytes.decode('ascii', 'ignore')  # Convert to ASCII and ignore non readible characters

					frames = bytes.split('<=>')  # Frame separator

					last_frame = frames[-1].split('#')[
					             :-1]  # Select the last frame, parse the fields (#) and discard the last value (EOF)
					print(last_frame)

					for field in last_frame:  # Iterate over the frame fields

						data = field.split(':')
						if len(data) < 2:
							# This is not a data field #
							pass
						else:
							# This is a data field #
							sensor_str = data[0]
							sensor_val = float(data[1])
							if sensor_str in sensor_keys:  # The sensor is in the available sensors #
								self.sensor_data[sensor_str] = sensor_val  # Update the sensor_data dict

					is_frame_ok = True

				except Exception as E:

					print("ERROR READING THE SENSOR. THIS IS NO GOOD!")
					print("The cause of the exception: " + E)
					self.serial.reset_input_buffer()

	def close(self):

		print("Cerrando base de datos!")
		self.database.close()  # Cerramos la DB
		print("Base de datos cerrada")
		print("Cerrando puerto serie!")
		self.serial.close()  # Cerramos la com. serie
		print("Puerto serie cerrado!")

	def __del__(self):

		print("Cerrando base de datos!")
		self.database.close()  # Cerramos la DB
		print("Base de datos cerrada")
		print("Cerrando puerto serie!")
		self.serial.close()  # Cerramos la com. serie
		print("Puerto serie cerrado!")
