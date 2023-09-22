# ASV Loyola-US project

Este paquete de ROS2 implementa los nodos necesarios para la ejecución de la rutina normal de funcionamiento de la flota de ASV.

Los nodos son los siguientes:

1. *Mission Node*:
    Nodo principal del sistema. Implementa los distintos modos de funcionamiento de la flota de ASV. Se encarga de la gestión de la misión, la comunicación con el *Ground Station* y la gestión de los nodos de control de los ASV. Este nodo llama a los servicios de los nodos de control para la ejecución de las distintas maniobras, como los de cambios de modo, el servidor de acción GoTo o el servidor de los sensores.

2. *Donekit Node*:
    No es más que un transductor de ROS a Mavlink a través de la librería Dronekit. Se encarga de la comunicación con el autopiloto de los ASV. Recibe los comandos de los nodos de control y los envía al autopiloto. Publica los datos de los sensores y del estado del ASV.

3. *MQTT Node*:
    Se encarga de la comunicación con el servidor. Recibe los comandos de la *Ground Station* y los envía al *Mission Node*. Publica los datos de los sensores y del estado del ASV en el servidor.

4. *Planner Node*:
    Se encarga del path planning de los ASV. Recibe los puntos de destino, y calcula la ruta a seguir sin obstáculos. Devuelve los puntos de la ruta al *Mission Node*. A este nodo se accede a través de un servicio.

5. *Sonar Node*:
    Se encarga de gestionar el sonar para dar medidas de batimetría.

## Dependencias

1. Es necesario instalar las siguientes dependencias:

```bash
    pip install bluerobotics-ping 
    pip install paho-mqtt
    pip install pyserial
    pip install dronekit
    pip install getmac
```

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
