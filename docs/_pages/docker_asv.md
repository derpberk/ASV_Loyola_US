---
layout: default
title: Docker ASV
description: Docker del vehículo
permalink: /docker_asv/
---

## Docker ASV

Este Docker se encarga del movimiento del vehículo, la adquisición de los datos de los sensores equipados y comunicación con el servidor. Para realizar el Docker nos basamos en un dockerfile propio donde copiamos el código de ASV [enlace](https://github.com/derpberk/ASV_Loyola_US), exponemos los puertos para la conexión con Navio2 y lanzamos el código para que se ejecute con un ros2 launch. El código del dockerfile se encuentra en el mismo repositorio, actualmente se encuentra dos disponible dos dockerfile tanto para sistema AMD y ARM.

Para el dockerfile ARM utilizamos la siguiente imagen 
```bash
arm64v8/ros:humble
```

Para el dockerfile AMD utilizamos la siguiente imagen 

```bash
osrf/ros:humble-desktop-full
```
luego establecemos el directorio donde vamos a trabajar en el docker 
```bash
WORKDIR /home/asv_workspace
```
establecemos las variables de entorno
```bash
ENV QT_X11_NO_MITSHM=1
```
Esta variable de entorno se refiere a la configuración del sistema de ventanas Qt para evitar el uso de la extensión MIT-SHM del sistema X11. La extensión MIT-SHM permite a las aplicaciones compartir memoria de forma eficiente para mejorar el rendimiento
```bash
ENV EDITOR=nano
```
Esta variable de entorno establece el editor de texto predeterminado que se utilizará en el contenedor
```bash
ENV XDG_RUNTIME_DIR=/tmp
```
Esta variable de entorno especifica el directorio donde se creará el directorio de tiempo de ejecución (runtime directory) de acuerdo con el estándar XDG (FreeDesktop.org). Este directorio se utiliza para almacenar archivos temporales relacionados con la sesión del usuario y otras aplicaciones durante el tiempo de ejecución. En este caso, se está configurando para que se utilice /tmp, que es un directorio comúnmente utilizado para archivos temporales en sistemas Unix/Linux.
```bash
ENV ROS_DOMAIN_ID=42
```
 Esta variable de entorno establece el ID de dominio de ROS (Robot Operating System) que se utilizará en el entorno del contenedor. ROS utiliza el concepto de dominios para separar los mensajes y los nodos entre diferentes redes o entornos. Establecer este valor en 42 asigna el ID de dominio 42 a todos los nodos de ROS que se ejecuten en el contenedor, lo que permite la comunicación adecuada entre ellos dentro del mismo dominio.

Realizamos un update del sistema en el docker e instalamos algunas herramientas para su uso si fuera necesario
```bash
RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    nano \
    python3-pip \
    python3-pydantic \
    ruby-dev \
    wget \
    xorg-dev \
    iputils-ping 

RUN pip3 install setuptools==58.2.0

Realizamos la instalación de las librerías necesarias para el funcionamiento de los códigos de los nodos

RUN pip3 install bluerobotics-ping 
RUN pip3 install paho-mqtt
RUN pip3 install pyserial
RUN pip3 install dronekit
RUN pip3 install getmac

# Esto es necesario por un error de la librería dronekit #
RUN sed -i "s/import collections/import collections.abc as collections/g" /usr/local/lib/python3.10/dist-packages/dronekit/__init__.py

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
```
Exponemos los puertos para poder realizar la comunicación con el Navio2 y el docker
```bash
EXPOSE 1883 1880 5688 5678
```
Copiamos el código del ASV desde la carpeta asv_workspace en nuestra carpeta /home/asv_workspace situada en el interior del docker.
```bash
COPY ./asv_workspace /home/asv_workspace
```
Realizamos un source de ROS2 para trabajar con él y realizar una build del codigo
```bash
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build"
```
Por último se lanza mediante comando, que al iniciarse el docker realice esta siguiente acción, donde realiza un "source" ROS2 y dependencias del código generado en la build generada en el paso anterior. Por último, realizamos un ros2 launch para donde ejecutamos el archivo system.launch.py donde se encuentra tanto los nodos como los parámetros para hacer funcionar nuestro ASV.
```bash
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch asv_loyola_us system.launch.py"]
```

Para lanzar el dockerfile se realiza mediante el siguiente comando
```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR --network bridge syanes/asv:arm64
```
docker run: Este comando se utiliza para ejecutar un contenedor Docker a partir de una imagen.

-it: Estas son dos opciones combinadas. -i o --interactive mantiene abierto el canal de comunicación estándar (STDIN) incluso si el contenedor no está adjunto, y -t o --tty asigna un pseudo-TTY (terminal) al contenedor.

--rm: Esta opción indica a Docker que elimine automáticamente el contenedor después de que se detenga la ejecución del comando. Esto ayuda a mantener limpio el sistema eliminando los contenedores que ya no se están utilizando.

--device /dev/SONAR --device /dev/SENSOR: Estas opciones agregan dispositivos específicos (/dev/SONAR y /dev/SENSOR) al contenedor. Esto permite que el contenedor tenga acceso directo a estos dispositivos, lo que puede ser necesario para interactuar con hardware específico conectado al sistema. Por ejemplo, SONAR podría ser un sonar utilizado para la detección submarina, y SENSOR podría ser algún otro tipo de sensor.

--network bridge: Esta opción especifica que el contenedor utilice la red bridge, que es la red predeterminada de Docker. La red bridge permite que el contenedor se comunique con otros contenedores y con el host Docker utilizando la configuración de red predeterminada.

syanes/asv:arm64: Esta es la imagen de Docker que se utilizará para ejecutar el contenedor. La imagen se identifica mediante su nombre (syanes/asv) y su etiqueta (arm64), que indica que esta es una imagen específica para la arquitectura ARM64.
