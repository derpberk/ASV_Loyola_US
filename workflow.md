MavROS trabaja con todo el protocolo mavlink adaptado a ROS en la rama https://github.com/mavlink/mavros/tree/ros2/mavros

En ella se encuentra toda la información de su instalación así como su uso. Se recomienda clonar la última versión de MavLink del repositorio oficial https://github.com/mavlink/mavlink

a 29/12/2021 MavROS en ROS2 presenta un problema en la gestión de sus topics y servicios debida a la cual la llamada de servicios o topics que conlleven una acción sobre el dispositivo (salvo los cmd) entran en un proceso de espera infinita debido a no recibir respuesta (https://github.com/mavlink/mavros/issues/1588)

Debido a este motivo se ha decidido utilizar MAVros para la lectura de sensores y crear un nodo usando dronekit para enviar acciones al dron, teniendo así 2 conexiones por mavlink en paralelo

los topics de lectura tienen el Namespace de MavROS, los topics de escritura tienen el Namespace de Dronekit, una vez se solucione el problema de MavROS en una actualización futura, será necesario cambiar dichos namespaces y eliminar el nodo de Dronekit
