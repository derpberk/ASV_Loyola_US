# ASV Loyola-US project

[![Docker Image CI](https://github.com/derpberk/ASV_Loyola_US/actions/workflows/docker-image.yml/badge.svg?branch=Production)](https://github.com/derpberk/ASV_Loyola_US/actions/workflows/docker-image.yml)

Este repositorio implementa el middleware con ROS2 y Docker para que los vehículos realicen tareas de monitorización. El sistema iplementa los nodos necesarios para la ejecución de la rutina normal de funcionamiento de la flota de ASV.

Los nodos son los siguientes:

1. *ASV Node*: Se encarga de gestionar los waypoints y de enviar las órdenes al controlador del ASV.

2. *Server Node*: Se encarga de la comunicación con el servidor.

3. *Sonar Node*: Se encarga de la lectura del sonar.

4. *WQP Node*: Se encarga de la comunicación con los sensors de calidad del agua.

## Ejecución en simulación

El simulador está implementado en una imagen de Docker. De esta forma, no dependemos de la instalación de SITL en el sistema. Para lanzar la simulación, ejecutamos el siguiente comando:

```bash
    docker pull syanes/asv_us:sitl
```

Si tenemos acceso al registry de bender:

```bash
    docker pull bender.us.es:5000/asv_us:sitl
```

### 1. *Lanzamiento del Contenedor de Docker SITL*

Para lanzar el contenedor de Docker con el simulador, ejecutamos el siguiente comando:

```bash
    docker run -it --rm --name sitl -p 5760:5760 -p 5798:5798 -P 5788:5788 syanes/asv_us:sitl
```

Usaremos el puerto 5798/tcp para conectarnos con Mission Planner y el puerto 5788/tcp para conectarnos con MAVROS.


### 2. *Lanzamiento de MAVROS*

Lanzaremos MAVROS con el siguiente comando para simular la comunicación con el controlador del ASV:

```bash
     ros2 launch mavros apm.launch fcu_url:="tcp://127.0.0.1:5788"
```

### 3. *Lanzamiento de los nodos*

Llamamos a ejecutar los nodos de la siguiente manera:

```bash
    ros2 launch asv_loyola_us system.launch.py
```


