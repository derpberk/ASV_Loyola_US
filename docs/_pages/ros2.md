---
layout: default
title: ROS2 Code
description: Código de ROS2
permalink: /ros2/
---

# ASV NODE

Este es un esquemático del funcionamiento del código de navegación y utilización de los sensores :

![Esquematico del codigo de ASV](/ASV_Loyola_US/images/ROS2scheme.png)

 
A continuación, se detalla una explicacion mas detallada del codigo de navegacion y utilizacion de los sensores: 

En este fragmento de código se encarga de inicializar parámetros, declarar servicios y declarar temas para el nodo asv_node:

```bash
    def initialize_parameters(self):

        # Get parameters from ROSPARAM
        self.declare_parameter('debug', True)
        self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('use_path_planner', True)
        self.use_path_planner = self.get_parameter('use_path_planner').get_parameter_value().bool_value

    def declare_services(self):

        # This node connects to the following services:
        # 1 - /mavros/set_mode mavros_msgs/srv/SetMode
        self.set_mode_point_client = self.create_client(SetMode, '/mavros/set_mode')
        # 2 - /path_planner_service asv_interfaces/srv/PathPlanner
        self.path_planner_client = self.create_client(PathPlanner, '/ASV/path_planner_service')
        # 3 - /mavros/mission/clear mavros_msgs/srv/WaypointClear
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        # 4 - /mavros/mission/push mavros_msgs/srv/WaypointPush
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')

    def declare_topics(self):

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        # Subscribe to the state of the vehicle /mavros/state
        self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_topic_callback, qos_profile)
        # Subscribe to the WP reached topic /mavros/mission/reached
        self.wp_reached_subscriber = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.wp_reached_topic_callback, qos_profile)
        # Subscribe to the battery topic /mavros/battery
        self.battery_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_topic_callback, qos_profile_BEF)
        # Subscribe to the start_asv topic /start_asv
        self.start_asv_subscriber = self.create_subscription(Bool, '/start_asv', self.start_asv_callback, qos_profile)
        # Subscribe to the wp_target topic /wp_target (FROM MQTT)
        self.wp_target_subscriber = self.create_subscription(GlobalPositionTarget, '/wp_target', self.wp_target_callback, qos_profile)
        # Subscribe to the wp_clear topic /wp_clear (FROM MQTT)
        self.wp_clear_subscriber = self.create_subscription(Bool, '/wp_clear', self.wp_clear_callback, qos_profile)
        # Subscribe to the asv position topic /mavros/global_position/global
        self.asv_position_subscriber = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)
```
* initialize_parameters(): Este método inicializa los parámetros para el nodo ASV. Declara dos parámetros: 'debug' y 'use_path_planner', ambos con valores predeterminados de True. Luego, recupera los valores de los parámetros usando get_parameter() y los almacena en atributos correspondientes (self.use_path_planner).

* declare_services(): Este método declara los servicios que utilizará el nodo ASV. Crea instancias de cliente para cuatro servicios diferentes:
    /mavros/set_mode (de mavros_msgs/srv/SetMode)
    /ASV/path_planner_service (de asv_interfaces/srv/PathPlanner)
    /mavros/mission/clear (de mavros_msgs/srv/WaypointClear)
    /mavros/mission/push (de mavros_msgs/srv/WaypointPush)

* declare_topics(): Este método declara los temas a los que se suscribirá el nodo ASV. Crea suscripciones para:

    Estado del vehículo (/mavros/state)
    Llegada al punto de ruta (/mavros/mission/reached)
    Estado de la batería (/mavros/battery)
    Inicio ASV (/start_asv)
    Objetivo del punto de ruta (/wp_target)
    Borrar punto de ruta (/wp_clear)
    Posición ASV (/mavros/global_position/global)

Cada suscripción está asociada con una función de devolución de llamada que se ejecutará cuando se reciban nuevos mensajes en los temas respectivos. Además, se definen perfiles de calidad de servicio (QoS) para comunicación confiable y de mejor esfuerzo.


En este siguiente fragmento del código se definen varios métodos de devolución de llamada (callback) que se activarán cuando se actualicen ciertos temas en el nodo ASV. Además, en el método __init__, inicializa varios atributos y objetos necesarios para el funcionamiento del nodo.



```bash

    def asv_position_callback(self, msg):
        # This function is called when the asv position topic is updated
        #self.get_logger().info(f"ASV position: {msg.latitude}, {msg.longitude}")
        self.asv_position = msg

    def state_topic_callback(self, msg):
        # This function is called when the state topic is updated
        # self.get_logger().info(f"State MODE: {msg.mode}")
        self.asv_mode = msg.mode
        self.asv_armed = msg.armed

    def wp_reached_topic_callback(self, msg):
        # This function is called when the WP is reached
        self.get_logger().info(f"WP reached: {msg.wp_seq}")

        if msg.wp_seq == self.mission_length - 1:
            self.wp_reached = True
            self.get_logger().info(f"Last WP reached!")

    def battery_topic_callback(self, msg):
        # This function is called when the battery topic is updated
        self.battery = msg.voltage

    def start_asv_callback(self, msg):
        # This function is called when the start_asv topic is updated
        self.get_logger().info(f"ASV has been called to start!")
        self.start_asv = True
    
    def wp_target_callback(self, msg):
        # This function is called when the wp_target topic is updated
        self.get_logger().info(f"New target WP received: {msg.latitude}, {msg.longitude}")

        self.wp_queue.put(msg, block=False) # Put the WP in the queue without blocking the main thread

    def wp_clear_callback(self, msg):
        # Just put the flag to msg #
        self.get_logger().info(f"WP cleared!")
        self.wp_cleared = True


    def __init__(self):
        super().__init__('asv_node')
        
        # Declare some parameters #
        self.initialize_parameters()

        # Initialise the mission state
        self.wp_reached = False
        self.wp_cleared = False
        self.start_asv = False
        
        # The WP Queue
        self.wp_queue = queue.Queue() # To store the WPs
        self.FSM_STATE = "IDLE"
        self.asv_mode = "MANUAL"
        self.asv_armed = False

```

* asv_position_callback(self, msg): Este método se llama cuando se actualiza el tema de posición del ASV. Actualiza el atributo self.asv_position con el mensaje recibido.

* state_topic_callback(self, msg): Este método se llama cuando se actualiza el tema de estado. Actualiza los atributos self.asv_mode y self.asv_armed con los valores de modo y armado del mensaje recibido.

* wp_reached_topic_callback(self, msg): Este método se llama cuando se alcanza un punto de ruta (WP). Registra que el WP ha sido alcanzado y, si es el último WP de la misión, establece el atributo self.wp_reached en True.

* battery_topic_callback(self, msg): Este método se llama cuando se actualiza el tema de batería. Actualiza el atributo self.battery con el voltaje de la batería recibido en el mensaje.

* start_asv_callback(self, msg): Este método se llama cuando se inicia el ASV. Establece el atributo self.start_asv en True.

* wp_target_callback(self, msg): Este método se llama cuando se actualiza el objetivo del punto de ruta (WP). Registra el nuevo objetivo en la cola de puntos de ruta (self.wp_queue).

* wp_clear_callback(self, msg): Este método se llama cuando se borra un punto de ruta (WP). Establece el atributo self.wp_cleared en True.

* __init__(self): Este método es el constructor de la clase. Inicializa los parámetros del nodo, como los atributos de estado de la misión, la cola de puntos de ruta (self.wp_queue), y el estado de la máquina de estados finitos (FSM).

A continuación llegamos al main loop del nodo asv:


```bash
    def main_loop(self):
            # This is the main loop of the ASV node # 
            # It is basically a FMS

            # If the mode is in manual, to IDLE
            if not self.asv_mode in ['GUIDED','AUTO'] or not self.asv_armed:
                
                if self.counter % 50 == 0:
                    self.get_logger().warning(f"We are in {self.asv_mode} mode, and ASV is {'armed' if self.asv_armed else 'not armed'}. Going to IDLE.")
                
                self.FSM_STATE = 'IDLE'
                self.mission_length = 0 # Reset the mission length

            # If the WP is cleared, go to IDLE #
            if self.wp_cleared:
                
                self.FSM_STATE = 'WAIT_WP'
                self.wp_cleared = False
                self.wp_queue = queue.Queue() # Clear the queue
                self.mission_length = 0 # Reset the mission length

                # Call the clear service - Sincronously #
                future = self.wp_clear_client.call_async(WaypointClear.Request())
                # Wait for the response #
                rclpy.spin_until_future_complete(self, future)
                if future.result().success:
                    self.get_logger().info(f"WP cleared!")

                self.set_mode('GUIDED') # Change the mode to GUIDED
            
            # THE FSM STATES #

            if self.FSM_STATE == 'IDLE':
                # Wait HERE until the ASV is called to start #
                if self.start_asv:
                    self.get_logger().info(f"FSM to WAIT_WP state")
                    self.FSM_STATE = 'WAIT_WP'
                    self.start_asv = False  # Deactivate the flag

            elif self.FSM_STATE == 'WAIT_WP':
                # Wait HERe until a WP is received #

                # If the path planner is used, unpack the path into the WP queue #
        
                if self.wp_queue.qsize() > 0:

                    # If the path queue is not empty, get the objective #
                    wp_objective = self.wp_queue.get()

                    # 1) Configure the request #

                    if self.use_path_planner:
                        new_objective = PathPlanner.Request()
                        new_objective.origin.lat = self.asv_position.latitude
                        new_objective.origin.lon = self.asv_position.longitude
                        new_objective.destination.lat = wp_objective.latitude
                        new_objective.destination.lon = wp_objective.longitude

                        # 2) Call the path planner #
                        future = self.path_planner_client.call_async(new_objective)

                        # Wait for the response #
                        rclpy.spin_until_future_complete(self, future)
                        self.get_logger().info(f"Path planner answered")

                        # Get the response #
                        response = future.result()

                        if not response.success:
                            self.get_logger().info(f"Path planner was not successful.")
                            return
                    
                        final_path = response.path.path
                
                    else:
                        # If the path planner is not used, just put the WP as the only WP in the list#
                        final_path = [wp_objective]

                    # Conform a list of WPs #
                    wp_list = []
                    mission_request = WaypointPush.Request()
                    for point in final_path:
                        wp = Waypoint()
                        wp.x_lat = point.lat
                        wp.y_long = point.lon
                        wp.autocontinue = True
                        wp.command = 16
                        wp.frame = 0 # Global frame
                        wp_list.append(wp)
                    
                    mission_request.waypoints = wp_list
                    mission_request.start_index = 0

                    # Push the list of WPs to the vehicle #
                    future = self.wp_push_client.call_async(mission_request)

                    # spin until results
                    rclpy.spin_until_future_complete(self, future)

                    if future.result().success:
                        self.get_logger().info(f"New path injected!")
                    else:
                        self.get_logger().info(f"Could not inject the mission.")
                        return
                    
                    # Get the length of the mission #
                    self.mission_length = len(wp_list)
                    
                    # Change the state #
                    self.FSM_STATE = 'GO_TO_WP'

                else:
                    self.FSM_STATE = 'WAIT_WP'

            elif self.FSM_STATE == 'GO_TO_WP':
                # Initialize the mission #
                self.set_mode('AUTO')  # Set the mode to AUTO
                self.get_logger().info(f"Starting mission!")

                # Change the state #
                self.FSM_STATE = 'WAIT_TO_REACH'

            elif self.FSM_STATE == 'WAIT_TO_REACH':
                # Wait until the WP is reached or until the point is cleared #

                if self.wp_reached:

                    self.FSM_STATE = 'WAIT_WP'
                    self.wp_reached = False
                    self.mission_length = 0 # Reset the mission length
                    # Clear the mission #
                    future = self.wp_clear_client.call_async(WaypointClear.Request())
                    # Wait for the response #
                    rclpy.spin_until_future_complete(self, future)
                    # Change the mode to GUIDED #
                    self.set_mode('GUIDED')

                else:
                    self.FSM_STATE = 'WAIT_TO_REACH'
                

        def set_mode(self, mode):
            # This function changes the mode of the vehicle #

            # Create the request #
            request = SetMode.Request()
            request.custom_mode = mode

            # Send the request #
            future = self.set_mode_point_client.call_async(request)

            # Wait for the response #
            rclpy.spin_until_future_complete(self, future)

            if future.result():
                self.get_logger().info(f"Mode changed to {mode}")

            return future



    def main(args=None):
        #init ROS2
        rclpy.init(args=args)
        
        #start a class that servers the services
        asv_node = ASV_node()
        asv_node.destroy_node()

        #after close connection shut down ROS2
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
```

Este fragmento de código implementa el bucle principal del nodo ASV (Vehículo Submarino Autónomo). El bucle principal se encarga de manejar el estado del vehículo y las tareas asociadas con el seguimiento de los puntos de ruta (WP) y la ejecución de la misión. Aquí está la explicación detallada del código:

def main_loop(self):: Este método representa el bucle principal del nodo ASV. Esencialmente, implementa una Máquina de Estados Finitos (FSM) que controla el comportamiento del vehículo.
Si el modo es manual, ir a IDLE: Si el ASV no está en modo "GUIDED" o "AUTO" o si no está armado, el ASV se coloca en estado "IDLE". Esto significa que el ASV no está activamente ejecutando ninguna misión y está en espera.

Si se ha borrado el WP, ir a IDLE: Si el WP ha sido borrado (indicado por self.wp_cleared), el ASV se coloca en estado "WAIT_WP", lo que significa que está esperando a recibir un nuevo WP.

Estados de la MEF: Luego, se evalúa el estado actual de la FSM y se ejecuta el comportamiento correspondiente para cada estado.
IDLE: En este estado, el ASV espera hasta que se inicie la misión (start_asv).

* WAIT_WP: En este estado, el ASV espera a que se reciba un nuevo WP. Si se recibe un WP, el ASV calcula la ruta hacia ese WP y se cambia al estado "GO_TO_WP".
GO_TO_WP: En este estado, el ASV comienza la misión, estableciendo el modo de vuelo en "AUTO" y comenzando a seguir la ruta hacia el WP.

* WAIT_TO_REACH: En este estado, el ASV espera hasta que alcance el WP o hasta que se borre el WP. Una vez que se alcanza el WP, el ASV vuelve al estado "WAIT_WP".

set_mode(self, mode): Este método cambia el modo de vuelo del vehículo. Envía una solicitud al servicio set_mode con el nuevo modo especificado. Espera la respuesta y, si se cambia el modo correctamente, registra el cambio en el registro de eventos.
main(args=None): Esta es la función principal que inicializa el entorno ROS 2, crea una instancia del nodo ASV, ejecuta el bucle principal y, finalmente, cierra el entorno ROS 2 después de que se cierre la conexión.

# Path Planner Node

Esta parte del código define una clase PathPlannerNode que representa un nodo en ROS 2 para planificar rutas. Primero declararemos los parametros,servicios y topics que se vayan a uasr en el nodo:

```bash
	def parameters(self):

		# Timeout to compute a path
		self.declare_parameter('path_planner_timeout', 20.0)
		self.path_planner_timeout = self.get_parameter('path_planner_timeout').get_parameter_value().double_value	
		# Base obstacle map path
		self.declare_parameter('navigation_map_path', 'mapas/Alamillo95x216')
		self.navigation_map_path = self.get_parameter('navigation_map_path').get_parameter_value().string_value
		# Debug mode
		self.declare_parameter('debug', True)
		self.debug = self.get_parameter('debug').get_parameter_value().bool_value

		# binary dilation
		self.declare_parameter('binary_dilation', 2)
		self.binary_dilation_size = self.get_parameter('binary_dilation').get_parameter_value().integer_value

		# Load the map

		self.navigation_map = np.genfromtxt(self.navigation_map_path + 'plantilla.csv', dtype=int, delimiter=' ')
		if self.binary_dilation_size > 0:
			self.dilated_navigation_map = binary_dilation(self.navigation_map, np.ones((self.binary_dilation_size, self.binary_dilation_size)))
		else:
			self.dilated_navigation_map = self.navigation_map

		grid_xy_to_lat_long = np.genfromtxt(self.navigation_map_path + 'grid.csv', delimiter=';', dtype=str)

		self.navigation_map_lat_long = np.zeros((2, *self.navigation_map.shape))

		# Transform the map to a float for long and lat
		for i in range(self.navigation_map.shape[0]):
			for j in range(self.navigation_map.shape[1]):
				self.navigation_map_lat_long[0, i, j] = float(grid_xy_to_lat_long[i, j].split(',')[0])
				self.navigation_map_lat_long[1, i, j] = float(grid_xy_to_lat_long[i, j].split(',')[1])
		
		# print(self.navigation_map)
					

	def declare_services(self):
		
		# Create a service to compute a path
		self.path_planner_service = self.create_service(PathPlanner, 'path_planner_service', self.path_planner_callback)

	def declare_topics(self):

		# Subscribe to a topic to receive new obstacles
		self.obstacles_subscriber = self.create_subscription(Location, 'detected_obstacles', self.obstacles_callback, 10)

```


* Método parameters(self): Este método se encarga de inicializar los parámetros del nodo, como el tiempo de espera para calcular una ruta, la ruta al mapa de obstáculos base, el modo de depuración y el tamaño de la dilatación binaria. Luego carga el mapa de navegación desde un archivo CSV y, si es necesario, realiza una dilatación binaria en el mapa para tener en cuenta el tamaño de los obstáculos.

* Método declare_services(self): Este método declara los servicios que el nodo proporcionará. En este caso, crea un servicio llamado path_planner_service que se utilizará para solicitar la planificación de una ruta.

* Método declare_topics(self): Este método declara los temas a los que el nodo se suscribirá para recibir información. En este caso, crea una suscripción al tema detected_obstacles para recibir información sobre obstáculos detectados.

A continuación vemos el resto del código:

```bash

	def __init__(self):
		super().__init__('path_planner_node')

		# Declare parameters, services and topics
		self.parameters()

		# Declare the dijkstra object
		self.dijkstra = Dijkstra(self.dilated_navigation_map, 1)

		self.declare_services()	
		self.declare_topics()
		


	def obstacles_callback(self, msg):

		# Take the lat long and search the closest point in the map
		lat = msg.lat
		long = msg.lon

		x_pos, y_pos = self.lat_long_to_xy(lat, long)

		self.get_logger().info('New obstacle updated at X: %d, Y: %d', x_pos, y_pos)

		# Update the map
		self.navigation_map[x_pos, y_pos] = 1

	def lat_long_to_xy(self, lat, long):

		# Find the closest point in the map
		print(lat)
		distance_lat = np.abs(self.navigation_map_lat_long[0, :, :] - lat)
		distance_long = np.abs(self.navigation_map_lat_long[1, :, :] - long)

		# Find position of minimum distance
		position_lat = np.argmin(distance_lat).astype(int)
		position_lat = np.unravel_index(position_lat, self.navigation_map.shape)[0]

		position_long = np.argmin(distance_long).astype(int)
		position_long = np.unravel_index(position_long, self.navigation_map.shape)[1]

		return position_lat, position_long
	
	def xy_to_lat_long(self, x, y):

		return self.navigation_map_lat_long[0, x, y], self.navigation_map_lat_long[1, x, y]

	def path_planner_callback(self, request, response):

		# Compute the path
		
		start = (request.origin.lat, request.origin.lon)
		goal = (request.destination.lat, request.destination.lon)

	
		start_xy = self.lat_long_to_xy(start[0], start[1])
		goal_xy = self.lat_long_to_xy(goal[0], goal[1])

		self.get_logger().info('New objetive updated: START: {}, GOAL: {}'.format(start_xy, goal_xy))

		response = PathPlanner.Response()
		solution = self.dijkstra.planning(start_xy, goal_xy, self.path_planner_timeout)

		self.get_logger().info('Solution obtained!')

		if solution is None:
			response.success = False
		else:

			response.success = True
			response.path.path = []
			solution = reduce_path(solution, self.dilated_navigation_map).astype(int)

			for point in solution:
				lat, long = self.xy_to_lat_long(point[0], point[1])
				response.path.path.append(Location(lat=lat, lon=long))
			
			response.path.path_length = len(response.path.path)
			
		return response	






def main(args=None):
	rclpy.init(args=args)
	node = PathPlannerNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()


```


* Método __init__(self): Este método es el constructor de la clase PathPlannerNode. En él, se llama al constructor de la clase base Node y se inicializan los parámetros, servicios y temas necesarios para el nodo. También se crea un objeto de la clase Dijkstra para realizar la planificación de rutas.

* Método obstacles_callback(self, msg): Este método se llama cuando se recibe un mensaje en el tema detected_obstacles. Toma las coordenadas de latitud y longitud del mensaje y busca el punto más cercano en el mapa de navegación. Luego actualiza el mapa de navegación con la posición del obstáculo.

* Método lat_long_to_xy(self, lat, long): Este método convierte las coordenadas de latitud y longitud en coordenadas de la matriz del mapa de navegación. Utiliza la distancia mínima en latitud y longitud para encontrar la posición más cercana en el mapa.

* Método xy_to_lat_long(self, x, y): Este método convierte las coordenadas de la matriz del mapa de navegación en coordenadas de latitud y longitud.

* Método path_planner_callback(self, request, response): Este método se llama cuando se recibe una solicitud de servicio en path_planner_service. Calcula la ruta óptima desde el punto de origen hasta el punto de destino utilizando el algoritmo de Dijkstra. Luego, crea una respuesta que contiene la ruta calculada y su longitud.

Función main(args=None): Esta función es la función principal del script. Inicializa el nodo PathPlannerNode, lo hace girar y luego apaga el sistema de ROS 2 cuando termina.

En resumen, este código define un nodo en ROS 2 que planifica rutas utilizando el algoritmo de Dijkstra y responde a los cambios en los obstáculos detectados en el entorno.

# MQQT COMUNICATION

Esta parte del código define la clase ServerCommunicationNode, que representa un nodo en ROS 2 responsable de la comunicación con un servidor externo y el manejo de diversos mensajes y servicios. Aquí está la explicación del código:


```bash

    def initialize_parameters(self):

        # Declare some parameters #
        self.declare_parameter('internet_loss_timeout', 30.0)
        self.internet_loss_timeout = self.get_parameter('internet_loss_timeout').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "adress")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value
        self.declare_parameter('mqtt_user', "user")
        self.vehicle_id = get_asv_identity()
        self.mqtt_user = 'asv' + str(get_asv_identity())
        self.declare_parameter('mqtt_password', "password")
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value

    def declare_topics(self):

        # This node connects to the following topics
        # 1) The state of the vehicle /mavros/state
        # 2) The battery /mavros/battery
        # 3) The start_asv topic /start_asv
        # 4) The wp_target topic /wp_target
        # 5) The position of the ASV /asv_position
        # 6) The wp_clear topic /wp_clear

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_REL=rclpy.qos.QoSProfile(
			depth=1,
			reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
			durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
			history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
			)

        # Subscriptions
        self.asv_state_subscription = self.create_subscription(State, '/mavros/state', self.asv_state_callback, qos_profile)
        self.asv_battery_subscription = self.create_subscription(BatteryState, '/mavros/battery', self.asv_battery_callback, qos_profile_BEF)
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)
        self.asv_orientation_subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.asv_orientation_callback, qos_profile_BEF)

        # Subscriptions to the sensors
        self.wqp_sensor_subscription = self.create_subscription(SensorMsg, '/wqp_measurements', self.wqp_sensor_callback, qos_profile_BEF)
        # Subscriptions to the sonar
        self.sonar_sensor_subscription = self.create_subscription(SonarMsg, '/sonar_measurements', self.sonar_sensor_callback, qos_profile_BEF)
        # Subscriptions to trash point
        self.trash_point_subscription = self.create_subscription(TrashMsg, '/zed2i_trash_detections/trash_localization', self.trash_point_callback, qos_profile_REL)

        # Publications
        self.start_asv_publisher = self.create_publisher(Bool, '/start_asv', qos_profile)
        self.wp_target_publisher = self.create_publisher(GlobalPositionTarget, '/wp_target', qos_profile)
        self.wp_clear_publisher = self.create_publisher(Bool, '/wp_clear', qos_profile)

    def declare_services(self):
        """ Services that this node offers and subscribes to."""

        # Service to change the mode of the ASV
        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.camera_recording_client = self.create_client(StartSvoRec, '/zed/zed_node/start_svo_rec')
        self.camera_stop_recording_client = self.create_client(Trigger, '/zed/zed_node/stop_svo_rec')


```

* Método initialize_parameters(self): Este método inicializa los parámetros necesarios para el nodo. Declara parámetros como el tiempo de espera por pérdida de conexión a Internet, la dirección del servidor MQTT, el nombre de usuario y la contraseña para la conexión al servidor.
* Método declare_topics(self): Este método declara los temas a los que se suscribe y los temas que publica el nodo. Estos temas incluyen información sobre el estado del vehículo, el estado de la batería, la posición del ASV, los datos de los sensores, la orientación del ASV y la detección de basura. Además, este método define los perfiles de calidad de servicio (QoS) para cada suscripción y publicación.
* Método declare_services(self): Este método declara los servicios que ofrece el nodo. Incluye servicios para cambiar el modo del ASV y para controlar la grabación de la cámara.


A continuación veremos el resto del codigo:

```bash


    def __init__(self):
        super().__init__('communication_node')

        # Get parameters
        self.initialize_parameters()
        # Declare subscribers
        self.declare_topics()
        # Declare services
        self.declare_services()

        self.topic_identity = 'asv/asv' + str(self.vehicle_id)

        self.asv_mode = "MANUAL"
        self.battery = -1
        self.asv_position = {'latitude': 0, 'longitude': 0, 'heading': 0}


        # Declare MQTT topics
        topics = [self.topic_identity + '/start_asv', 
                self.topic_identity + '/wp_clear', 
                self.topic_identity + '/wp_target',
                self.topic_identity + '/asv_mode',
                self.topic_identity + '/camerarecord_on',
                self.topic_identity + '/camerarecord_off']
        
        for topic in topics:
            self.get_logger().info(f"Subscribing to {topic}")

        try:
            self.mqttConnection = MQTT(name=self.topic_identity, 
                                        addr=self.mqtt_addr, 
                                        user=self.mqtt_user, 
                                        password=self.mqtt_password, 
                                        on_message=self.on_message, 
                                        on_disconnect=self.on_disconnect,
                                        topics2suscribe=topics
                                        )
        except ConnectionRefusedError:
            self.get_logger().error(f"Connection to MQTT server was refused")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except OSError:
            self.get_logger().error(f"MQTT server was not found")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except TimeoutError:
            self.get_logger().error(f"MQTT was busy, timeout error")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except:
            error = traceback.format_exc()
            self.get_logger().fatal(f"MQTT connection failed, unknown error:\n {error}")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()

        # Declare timer for publishing data
        self.publishing_timer = self.create_timer(0.5, self.pub_timer_callback)

        # Spin forever
        while rclpy.ok():
            rclpy.spin_once(self)
            sleep(0.1)


    def pub_timer_callback(self):
        # When the timer is triggered, publish the data

        # Publish the complete state of the ASV
        self.mqttConnection.send_new_msg(json.dumps({
            'mode': self.asv_mode,
            'battery': self.battery,
            'Latitude': self.asv_position['latitude'],
            'Longitude': self.asv_position['longitude'],
            'Heading': self.asv_position['heading'],
            'veh_num': self.vehicle_id,
            'date': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }), topic = self.topic_identity + '/asv_state')

    def on_disconnect(self,  client,  __, _):

        sleep(1)
        self.get_logger().error("connection to server was lost")

        if not ping_google():
            time_without_internet = 0

            while not ping_google():

                self.get_logger().error("no internet connection, waiting for internet",once=True)
                sleep(1)
                time_without_internet += 1

                if time_without_internet >= self.internet_loss_timeout:
                    self.processing == True
                    self.call_msg.asv_mode = 3 #if we waited for too long, change into manual mode

            self.get_logger().info("Internet connection regained")
        else:
            self.get_logger().error("There is internet, unknown reason, retrying to connect to MQTT")

    def on_message(self, _client, _, msg):
        # This function is called when a message is received

        # Get the topic
        topic = msg.topic
        # Get the payload
        payload = msg.payload.decode("utf-8")

        # Check the topic
        if topic == self.topic_identity + '/start_asv':
            # If the topic is /asv_start, send the start command trough the topic
            msg = Bool()
            msg.data = payload == 1
            self.start_asv_publisher.publish(msg)
            self.get_logger().info("Start ASV command received")

        elif topic == self.topic_identity + '/wp_clear':
            # If the topic is /wp_clear, send the clear command trough the topic
            msg = Bool()
            msg.data = payload == 1
            self.wp_clear_publisher.publish(msg)
            self.get_logger().info("Clean WPs command received")

        elif topic == self.topic_identity + '/wp_target':
            # If the topic is /wp_target, send the WP command trough the topic
            # Format the payload to a dict (From json)
            payload = json.loads(payload)
            self.get_logger().info("New WP received: " + str(payload))
            msg = GlobalPositionTarget()
            try:
                msg.latitude = payload['latitude']
                msg.longitude = payload['longitude']
                # Publish the WP
                self.wp_target_publisher.publish(msg)
            except KeyError:
                self.get_logger().error("The payload of the requested WP is not correct")

        elif topic == self.topic_identity + '/asv_mode':
            # Call the service to change the mode of the ASV
            # Check if the mode is correct
            # Create the request
            request = SetMode.Request()
            request.custom_mode = payload
            # Call the service
            self.set_mode_srv.call_async(request)
            self.get_logger().info("Change mode command received: " + payload)
        
        elif topic == self.topic_identity + '/camerarecord_on':
            current_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            # Construct the filename with the current date
            filename = f"/root/CameraRecord/ASV_{current_date}.svo"
            self.message_zed = {
                    'svo_filename': filename,
                    'compression_mode': 2,
                    'target_framerate': 30,
                    'bitrate': 6000
                }
            self.get_logger().info("start record")
            call_service_extern(self, self.camera_recording_client, self.message_zed)
        
        elif topic == self.topic_identity + '/camerarecord_off':
            self.message_stop={}
            self.get_logger().info("stop record")
            call_service_extern(self, self.camera_stop_recording_client, self.message_stop)
        else:
            self.get_logger().error("The topic " + topic + " is not recognized")
            self.get_logger().error("The payload is " + str(payload))

    def asv_state_callback(self, msg):
        # This function is called when the state topic is updated
        self.asv_mode = msg.mode
    
    def asv_battery_callback(self, msg):
        # This function is called when the battery topic is updated
        self.battery = msg.percentage

    def asv_position_callback(self, msg):

        self.asv_position['latitude'] = msg.latitude
        self.asv_position['longitude'] =  msg.longitude

    def asv_orientation_callback(self, msg):
         
         euler = quaternion_to_euler([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
         self.asv_position['heading'] = euler[2]

    def wqp_sensor_callback(self, msg):
        # This function is called when the wqp_sensor topic is updated
        # Check if the message is correct
        if msg.success:
            # If the message is correct, send the message to the MQTT server

            # Format the json msg
            json_msg = json.dumps({
                'conductivity': msg.conductivity,
                'temperature_ct': msg.temperature_ct,
                'turbidity': msg.turbidity,
                'ph': msg.ph,
                'vbat': msg.vbat,
                'lat': msg.lat,
                'lon': msg.lon,
                'date': msg.date,
                'veh_num': self.vehicle_id
            })

            # Send the message
            self.mqttConnection.send_new_msg(json_msg, topic = '/database/wqp')
        else:
            self.get_logger().error("The message received from the WQP sensor is not correct")

    def sonar_sensor_callback(self, msg):
        # This function is called when the sonar_sensor topic is updated
        if msg.success:
            
            json_msg = json.dumps({
                'measurement': msg.distance,
                'Latitude': msg.lat,
                'Longitude': msg.lon,
                'veh_num': self.vehicle_id,
                'date': msg.date
            })

            self.mqttConnection.send_new_msg(json_msg, topic = '/database/sonar')
        else:
            self.get_logger().error("The message received from the sonar sensor is not correct")

    def trash_point_callback(self, msg):
        # This function is called when the sonar_sensor topic is updated
        if msg.success:
            if not isinstance(msg.lat,(int, float)) or numpy.isnan(msg.lat) :
                msg.lat=0.0
                msg.lon=0.0

            json_msg = json.dumps({
                'Latitude': msg.lat,
                'Longitude': msg.lon,
                'veh_num': self.vehicle_id,
                'date': msg.date
            })

            self.mqttConnection.send_new_msg(json_msg, topic = '/database/trash')
        else:
            self.get_logger().error("The message received from the sonar sensor is not correct")

def main(args=None):
    #init ROS2
    rclpy.init(args=args)

    #start a class that servers the services
    server_comms_node = ServerCommunicationNode()
    server_comms_node.destroy_node()

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

* Inicialización del nodo (__init__): En este método se inicializa el nodo con el nombre "communication_node". Luego se obtienen los parámetros necesarios mediante el método initialize_parameters(). A continuación, se declaran los suscriptores (topics a los que se suscribe el nodo) y los servicios que ofrece el nodo, utilizando los métodos declare_topics() y declare_services(), respectivamente. 

* Además, se establecen algunas variables de estado del ASV, como el modo, la batería y la posición. Se crea una instancia de la clase MQTT para la comunicación MQTT, y se configuran los tópicos MQTT a los que se suscribe el nodo.

* Método pub_timer_callback(self): Este método es un callback que se llama cada vez que se activa el temporizador de publicación. Su función es publicar el estado completo del ASV a través de MQTT, incluyendo el modo, la batería, la posición y la fecha y hora actual.

* Métodos on_disconnect(self) y on_message(self): Estos métodos manejan la desconexión del cliente MQTT y la recepción de mensajes MQTT, respectivamente. on_disconnect() se encarga de reconectar al cliente MQTT si se pierde la conexión o la conexión a Internet. on_message() procesa los mensajes MQTT recibidos y realiza acciones correspondientes en función del tópico y el payload recibidos.

* Métodos asv_state_callback(self), asv_battery_callback(self), asv_position_callback(self), asv_orientation_callback(self), wqp_sensor_callback(self), sonar_sensor_callback(self), y trash_point_callback(self): Estos métodos son callbacks que se llaman cuando los suscriptores reciben mensajes en los topics correspondientes. Cada uno de estos métodos actualiza la información del ASV según los datos recibidos.

* Función main(): Esta función inicia ROS 2, crea una instancia de la clase ServerCommunicationNode, y luego detiene ROS 2 después de cerrar la conexión.

En resumen, este código implementa un nodo en ROS 2 que facilita la comunicación entre el ASV y un servidor externo a través de MQTT, y procesa mensajes relacionados con el estado del ASV, la batería, la posición, y otros datos de sensores.


# SONAR NODE 

Este código define una clase llamada Sonar_node, que representa un nodo en ROS 2 encargado de la comunicación con el sensor sonar de BlueRobotics.

```bash

    def parameters(self):

        self.declare_parameter('sonar_USB_string', '/dev/SONAR')
        self.sonar_USB_string = self.get_parameter('sonar_USB_string').get_parameter_value().string_value

        self.declare_parameter('sonar_baudrate', 115200)
        self.baudrate = self.get_parameter('sonar_baudrate').get_parameter_value().integer_value

        self.declare_parameter('debug', True)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('sonar_measurement_frequency', 1.0)
        self.sonar_measurement_frequency = self.get_parameter('sonar_measurement_frequency').get_parameter_value().double_value
        

    def declare_topics(self):

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create a publisher for the sonar measurements
        self.sonar_publisher = self.create_publisher(SonarMsg, '/sonar_measurements', qos_profile_BEF)
        self.sonar_publisher_timer = self.create_timer(self.sonar_measurement_frequency, self.sonar_publish)
       
        # Create a subscription to the position of the ASV
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)

    def asv_position_callback(self, msg):

        self.position[0] = msg.latitude
        self.position[1] = msg.longitude

    def connect_to_sonar(self):

        connection_trials = 0
        
        while True:
            # Try sonar connection
            self.ping_device = Ping1D()
            self.ping_device.connect_serial(self.sonar_USB_string, self.baudrate)

            if self.ping_device.get_ping_enable:
                self.get_logger().info(f"Sonar connected!")
                return True
                break
            else:
                self.get_logger().info(f"Sonar not connected! Trial: {connection_trials}")
                connection_trials += 1
                time.sleep(1)

            if connection_trials > 10:
                self.get_logger().info(f"Failed to connect to Sonar")
                break
        
        return False

```


* Método parameters(self): Este método se encarga de inicializar los parámetros necesarios para el nodo. Define y declara varios parámetros, como la cadena USB del sonar (sonar_USB_string), la velocidad de transmisión (baudrate), un indicador de depuración (debug), y la frecuencia de medición del sonar (sonar_measurement_frequency). Estos parámetros se obtienen utilizando el método get_parameter().

* Método declare_topics(self): Este método declara los topics que el nodo va a publicar y suscribir. Crea un publisher para publicar las mediciones del sonar en el topic "/sonar_measurements" utilizando el tipo de mensaje SonarMsg. También crea una suscripción al topic "/mavros/global_position/global" para recibir la posición del ASV (vehículo submarino autónomo) utilizando el tipo de mensaje NavSatFix. Ambos topics se crean con un perfil de calidad de servicio (QoS) configurado como BEST_EFFORT.

* Método asv_position_callback(self, msg): Este método es un callback que se llama cuando se recibe un mensaje en el topic "/mavros/global_position/global". Actualiza la posición del ASV utilizando los datos recibidos en el mensaje.

* Método connect_to_sonar(self): Este método intenta establecer una conexión con el sonar. Utiliza un bucle while True para intentar conectarse repetidamente. En cada iteración del bucle, intenta conectar con el sonar utilizando la biblioteca Ping1D y el método connect_serial(). Si la conexión tiene éxito (get_ping_enable devuelve verdadero), se registra un mensaje de éxito y se devuelve True. Si la conexión falla después de 10 intentos, se registra un mensaje de error y se devuelve False.


``` bash
    
    def __init__(self):
        super().__init__("sonar_service")

        self.parameters()

        self.declare_topics()

        self.sonar_msg = SonarMsg()

        self.position = [0.0, 0.0]

        self.number_of_bad_readings = 0
    

        if self.DEBUG:
            self.get_logger().info(f"Simulating sonar measurements")

        if  not self.DEBUG:

            if self.connect_to_sonar():
                self.get_logger().info(f"Sonar connected")
            else:
                self.get_logger().info(f"Sonar not connected")
                self.free_resources()
                sys.exit(1)
            

    def destroy_usb(self):
        if self.ping_device:
            self.ping_device.close()

    def generate_fake_data(self):

        displaced_lat = self.position[0] - 37.418691117644244
        displaced_long = self.position[1] - 6.001191255201682
        interval_lat = abs(37.418716586727506 - 37.418691117644244)
        interval_long = abs(6.001191255201682 - 6.0007770870275252)
        self.sonar_msg.success = True
        self.sonar_msg.distance = 30 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
        self.sonar_msg.confidence = 1.0
        self.sonar_msg.lat = self.position[0]
        self.sonar_msg.lon = self.position[1]
        self.sonar_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        return self.sonar_msg

    def sonar_publish(self):

        if not self.DEBUG:

            if self.ping_device and self.ping_device.get_ping_enable: #Si estamos concetados realizamos el checkeo

                data = self.ping_device.get_distance()

                if data is None:

                    self.sonar_msg.distance = -1.0
                    self.sonar_msg.confidence = -1.0
                    self.sonar_msg.success = False
                    self.get_logger().info(f"Bad reading from sonar - None Received")
                    self.number_of_bad_readings += 1

                    if self.number_of_bad_readings > 10:
                        self.get_logger().info("Sonar not working - Trying to reconnect")
                        self.ping_device.close()
                        self.connect_to_sonar()
                else:
                    # Hooraay! We have a good reading

                    self.number_of_bad_readings = 0
                    self.sonar_msg.distance = float(data["distance"])
                    self.sonar_msg.confidence = float(data["confidence"])
                    self.sonar_msg.success = True

            self.sonar_msg.lat = self.position[0]
            self.sonar_msg.lon = self.position[1]
            self.sonar_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")


        else:
            # We are simulating the sonar
            self.sonar_msg = self.generate_fake_data()
        
        # Send the message
        self.sonar_publisher.publish(self.sonar_msg)
    

    def free_resources(self):
        """ Free the serial port resources """
        self.destroy_usb()

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    sonar_node = Sonar_node()
    rclpy.spin(sonar_node)

    # Destroy the node explicitly
    sonar_node.free_resources()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```


* Método __init__(self): Este método es el constructor de la clase Sonar_node. Se llama automáticamente cuando se crea un objeto de esta clase. Dentro del constructor, se llama al constructor de la clase base (super().__init__("sonar_service")) para inicializar el nodo con el nombre "sonar_service". Luego se inicializan los parámetros del nodo llamando al método parameters(), se declaran los topics llamando al método declare_topics(), se inicializan algunas variables de estado (self.sonar_msg, self.position, self.number_of_bad_readings), y se verifica si el modo de depuración (DEBUG) está activado o no. Si no está en modo de depuración (not self.DEBUG), intenta conectarse al sonar llamando al método connect_to_sonar(). Si la conexión es exitosa, se registra un mensaje de éxito; de lo contrario, se registra un mensaje de error y se liberan los recursos antes de salir del programa.

* Método destroy_usb(self): Este método se encarga de liberar los recursos del puerto USB del sonar. Se llama cuando el nodo se está apagando o cerrando.

* Método generate_fake_data(self): Este método genera datos simulados del sonar. Calcula la distancia simulada basada en la posición actual del ASV y devuelve un mensaje SonarMsg simulado con los datos generados.

* Método sonar_publish(self): Este método se encarga de publicar los datos del sonar. Si el nodo no está en modo de depuración (not self.DEBUG), verifica si está conectado al sonar y obtiene las mediciones del sonar utilizando el método get_distance() de la biblioteca del sonar. Si las mediciones son válidas, actualiza el mensaje SonarMsg con los datos del sonar y lo publica en el topic /sonar_measurements. Si el nodo está en modo de depuración, genera datos simulados del sonar y los publica.

* Método free_resources(self): Este método libera los recursos del sonar. Llama al método destroy_usb() para cerrar el puerto USB del sonar.

* Función main(args=None): Esta función inicializa ROS 2, crea un objeto de la clase Sonar_node, hace girar el nodo (rclpy.spin(sonar_node)), libera los recursos del sonar y luego apaga ROS 2 llamando a rclpy.shutdown().

# SENSOR NODE



```bash
    
    #his functions defines and assigns value to the
    def parameters(self):


        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('wqp_sensor_USB_string', "/dev/SENSOR")
        self.USB_string = self.get_parameter('wqp_sensor_USB_string').get_parameter_value().string_value

        self.declare_parameter('wqp_sensor_baudrate', 115200)
        self.baudrate = self.get_parameter('wqp_sensor_baudrate').get_parameter_value().integer_value

        self.declare_parameter('wqp_sensor_timeout', 10.0)
        self.timeout = self.get_parameter('wqp_sensor_timeout').get_parameter_value().double_value

        self.declare_parameter('wqp_sensor_measurement_frequency', 1.0)
        self.measurement_frequency = self.get_parameter('wqp_sensor_measurement_frequency').get_parameter_value().double_value
    
    def declare_topics(self):
        
        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

         # Create a subscription to the position of the ASV
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)

        # Create a publisher for the sensor measurements
        self.sensor_publisher = self.create_publisher(SensorMsg, '/wqp_measurements', qos_profile_BEF)
        self.sensor_publisher_timer = self.create_timer(self.measurement_frequency, self.sensor_publish)
    

    def asv_position_callback(self, msg):
        # Save the position of the ASV
        self.position[0] = msg.latitude
        self.position[1] = msg.longitude

    def generate_fake_data(self):
        # Generate fake data

        sensor_measurement = SensorMsg()

        sensor_measurement.success = True
        sensor_measurement.conductivity = 0.0
        sensor_measurement.temperature_ct = 0.0
        sensor_measurement.turbidity = 0.0
        sensor_measurement.ph = 0.0
        sensor_measurement.vbat = 0.0
        sensor_measurement.lat = self.position[0]
        sensor_measurement.lon = self.position[1]
        sensor_measurement.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        return sensor_measurement

```

Método parameters(self): Este método se encarga de inicializar los parámetros necesarios para el funcionamiento del módulo. Declara y asigna valores a los parámetros relacionados con la configuración del sensor, como el puerto USB del sensor (wqp_sensor_USB_string), la velocidad de transmisión (wqp_sensor_baudrate), el tiempo de espera (wqp_sensor_timeout), y la frecuencia de medición (wqp_sensor_measurement_frequency). Además, declara el parámetro debug para habilitar o deshabilitar el modo de depuración.

* Método declare_topics(self): Este método se encarga de declarar los topics a los que el módulo se suscribe y a los que publica. Crea una suscripción al topic /mavros/global_position/global para recibir la posición del ASV (Sistema de Vehículo Autónomo). También crea un publicador para publicar las mediciones del sensor en el topic /wqp_measurements.

* Método asv_position_callback(self, msg): Este método es un callback que se ejecuta cuando se recibe un mensaje de posición del ASV. Guarda la posición del ASV en una variable de clase llamada position.

* Método generate_fake_data(self): Este método genera datos simulados del sensor. Crea un objeto SensorMsg y asigna valores ficticios para las mediciones de conductividad, temperatura, turbidez, pH, voltaje de la batería, así como la latitud, longitud y fecha de la medición. Estos datos simulados se utilizan para pruebas o depuración cuando el modo de depuración (debug) está habilitado.


```bash
    

    def __init__(self):

        #start the node
        super().__init__('wqp_sensor_node')
        
        # declare parameter of drone IP
        self.parameters()
        self.declare_topics()

        self.sensor_msg = SensorMsg()

        self.position = [0.0, 0.0]

        self.pattern = r'data=([^,]+),([^,\]]+)'

        if self.DEBUG:
            self.get_logger().warning("Debug mode enabled")

        if  not self.DEBUG:
            connection_trials = 0
            while True:
                # Try sensor connection
                self.serial = serial.Serial(self.USB_string,self.baudrate, timeout=10)
                
                if self.serial.open:
                    self.get_logger().info(f"Sensor connected!")
                    break
                else:
                    self.get_logger().info(f"Sensor not connected! Trial: {connection_trials}")
                    connection_trials += 1
                    time.sleep(1)

                if connection_trials > 10:
                    self.get_logger().info(f"Failed to connect to Sensor")
                    break
        
    
    def read_sensor(self):
        
        # Send the sensor a command to take a sample, and read the response
        read_trials = 0
        read_ok = False

        # First, check if the sensor is connected
        if self.serial.is_open:

            # Wait for the response 100ms
            read_data = ""
            time.sleep(0.1)

            # Try 5 times 
            while(read_trials < 5 or not read_ok):
                    
                    # Flush the input buffer
                    self.serial.read_all()
                    
                    # Send the command to the sensor
                    self.serial.write(bytes("mscan\n",'ascii'))

                    # Wait for the response
                    time.sleep(0.1)

                    # Increment the number of trials
                    read_trials += 1
                    
                    # Read the incoming data byte by byte
                    while(self.serial.in_waiting > 0):

                        new_incoming_data = self.serial.read() # Read one byte

                        try:
                            decoded_data = new_incoming_data.decode()
                            new_character = str(decoded_data)
                        except:
                            self.get_logger().debug(f"Cannot decode incomming byte!")
                            continue
                            
                        # Append the new character to the read data
                        read_data += new_character
    
                        if len(read_data) > 0 and '}' in read_data:
                            # We have read the whole message
                            read_ok = True
                            self.get_logger().debug(f"A message has been read from the sensor: {read_data}")
                            break
            

            if read_ok:
                return read_data
            elif read_trials >= 5:
                self.get_logger().info(f"Too many trials to read the sensor!")
                return None
            else:
                return None


    def status_suscriber_callback(self, msg):
        self.status=msg

    def destroy_usb(self):
        if self.serial.open:
            self.serial.close

    def reconnect_sensor(self):

        # Close the serial port
        if self.serial.is_open:
            self.serial.close() # This is important
        
        del self.serial # This is important

        # Try to reconnect
        connection_trials = 0

        while connection_trials < 10:

            # Try sensor connection
            self.serial = serial.Serial(self.USB_string, self.baudrate, timeout=10)
            
            if self.serial.open:
                self.get_logger().info(f"Sensor connected!")
                return True
            else:
                self.get_logger().info(f"Sensor not connected! Trial: {connection_trials}")
                connection_trials += 1
                time.sleep(0.1)

            if connection_trials > 10:
                self.get_logger().info(f"Failed to connect to Sensor")
                return False
    

    def sensor_publish(self):

        if not self.DEBUG:

            data = self.read_sensor()

            # Check if the data is not empty
            if data is None:

                self.get_logger().info(f"Sensor data is empty!")
                # Try to reconnect to the sensor
                reconnect_response = self.reconnect_sensor()

                # If the reconnection is successful, try to read the sensor again
                if reconnect_response:
                    data = self.read_sensor()
                else:
                    self.get_logger().info(f"Failed to reconnect to the sensor!")
                    return

            # Now we have the data, we can parse it
                
            self.get_logger().info(data)

            self.sensor_msg.success=True
            
            # Find all matches of the pattern in the input string
            matches = re.findall(self.pattern, data)
            
            if len(matches) == 0:
                self.get_logger().info(f"No matches found in the sensor data")
                return

            for match in matches:

                sensor_str = match[0]
                sensor_val = match[1]

                if sensor_str == "Cond":
                    #self.get_logger().info(f"Found Conductivity {sensor_val}")
                    self.sensor_msg.conductivity = float(sensor_val)
                if sensor_str == "TempCT":
                    #self.get_logger().info(f"Found Temperature from an CT.X2 sensor {sensor_val}")
                    self.sensor_msg.temperature_ct = float(sensor_val)
                if sensor_str == "Turbidity":
                    #self.get_logger().info(f"Found Turdibity {sensor_val}")
                    self.sensor_msg.turbidity = float(sensor_val)
                if sensor_str == "pH":
                    #self.get_logger().info(f"Found pH value {sensor_val}")
                    self.sensor_msg.ph = float(sensor_val)
                if sensor_str == "vbat":
                    #self.get_logger().info(f"Found battery value {sensor_val}")
                    self.vbat = float(sensor_val)

            self.sensor_msg.vbat = (62.5*(self.vbat)-425)
            self.sensor_msg.lat = self.position[0]
            self.sensor_msg.lon = self.position[1]
            self.sensor_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")  
         
        else:
            
            # We are simulating the sensor
            self.sensor_msg = self.generate_fake_data()
        

        # Send the message
        self.sensor_publisher.publish(self.sensor_msg)
            

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    sensor_module = WQP_Sensor_module()
    rclpy.spin(sensor_module)

    # Destroy the node explicitly
    sensor_module.destroy_usb()
    rclpy.shutdown()





if __name__ == '__main__':
    main()


```


Se inicializan los parámetros del sensor mediante el método parameters() y se declaran los topics mediante el método declare_topics().

Se inicializan algunas variables, como sensor_msg para almacenar los datos del sensor, position para almacenar la posición y pattern para definir el patrón de datos del sensor.
Se comprueba si el modo de depuración (DEBUG) está habilitado. Si no está en modo de depuración, se intenta conectar al sensor mediante un bucle while.

* Método read_sensor(self): Este método se encarga de enviar un comando al sensor para tomar una muestra y leer la respuesta del sensor. Utiliza un bucle while para intentar leer la respuesta del sensor hasta cinco veces. Si no se puede leer la respuesta después de cinco intentos, se considera un fallo en la lectura.

* Método status_suscriber_callback(self, msg): Este método es un callback que se ejecuta cuando se recibe un mensaje de estado de suscripción. Este método guarda el estado en la variable status.

* Método destroy_usb(self): Este método se encarga de cerrar el puerto serial del sensor.

* Método reconnect_sensor(self): Este método intenta reconectar al sensor cerrando el puerto serial y abriendo uno nuevo. Utiliza un bucle while para intentar la reconexión hasta diez veces.

* Método sensor_publish(self): Este método se encarga de publicar los datos del sensor. Utiliza el método read_sensor() para obtener los datos del sensor. Si el modo de depuración está deshabilitado, parsea los datos del sensor y los publica en el topic correspondiente.

* Función main(args=None): Esta función inicializa ROS 2, crea una instancia de la clase WQP_Sensor_module, ejecuta el bucle principal (spin) y luego destruye explícitamente el nodo y cierra ROS 2 al finalizar la ejecución.




