Se recomienda haber completado los tutoriales y leído la documentación de https://docs.ros.org/en/foxy/index.html

Se ha creado un Paquete {asv_interfaces} para almacenar todos los mensajes, servicios y acciones entre nodos, Todos los paquetes deben tenerlo como referencia para usarlos

Se ha creado un paquete {asb_loyola_us} donde correrán todos los nodos que tomen parte en el programa

Para simulación {simulator} contiene nodos que ayudan a la publicación de datos. Este paquete requiere tener Ardupilot instalado para usar SITL

Launch=
	"system.launch.py" lanza una instancia de ASV
	"nodes.launch.py" define todos los nodos/programas que van a participar en el ASV
	
	Estan separados para hacer más corto el código launch y poder lanzar la instancia una vez la Raspberry nos envíe su identidad
	
config= 
	"config.yaml" esta formado por una serie de parámetros de ROS que definen al ASV
	


Nodos=
	"mission" recreación de main_as service con ciertos cambios
	"planner" realiza un algoritmo de planificación
	"watchdog" realiza gestión de errores y bloqueos del programa principal
	"dronekit" realiza las tareas de comunicación con la Raspberry
	"mqtt" #deprecated# realiza las tareas de comunicación con el servidor
	
	
	

