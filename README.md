# ASV Loyola-US project

[![Docker Image CI](https://github.com/derpberk/ASV_Loyola_US/actions/workflows/docker-image.yml/badge.svg?branch=Production)](https://github.com/derpberk/ASV_Loyola_US/actions/workflows/docker-image.yml)

Este repositorio implementa el middleware con ROS2 y Docker para que los vehículos realicen tareas de monitorización. El sistema iplementa los nodos necesarios para la ejecución de la rutina normal de funcionamiento de la flota de ASV.

Los nodos son los siguientes:

1. *ASV Node*: Se encarga de gestionar los waypoints y de enviar las órdenes al controlador del ASV.

2. *Server Node*: Se encarga de la comunicación con el servidor.

3. *Sonar Node*: Se encarga de la lectura del sonar.

4. *WQP Node*: Se encarga de la comunicación con los sensors de calidad del agua.

## Ejecución en simulación

### 1. *Instalación del Ardupilot SITL* 

Lo primero que haremos será instalar Ardupilot SITL para simular el vehículo. Hay que seguir las instrucciones de la [página oficial](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).

### 2. *Lanzamiento de Ardupilot SITL*

Lanzaremos Ardupilot SITL con el siguiente comando para simular el vehículo en el Lago del Alamillo:

```bash 
    sim_vehicle.py -v Rover -f sailboat-motor --console --map \
    --out=127.0.0.1:14550 \
    -l "37.4187024996518,-6.0011815072283445,0.0,0.0" 
```

### 3. *Lanzamiento de los nodos*

Llamamos a ejecutar los nodos de la siguiente manera:

```bash
    ros2 launch asv_loyola_us system.launch.py
```

Si en config.yaml se ha puesto el parámetro *debug* a *true*, el sistema funcionará con el simulador (*127.0.0.1:1455.*). Si se pone a *false*, el sistema funcionará con el ASV real y esperará encontrar a Navio2 en la dirección *tcp:navio.local:5678*.
