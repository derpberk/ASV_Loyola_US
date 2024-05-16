---
layout: default
title: Docker ZED
description: Docker de la Cámara ZED
permalink: /docker_zed/
---

# Docker ROS-Wrapper-Zed

Este docker se encarga del funcionamiento de la cámara ZED2i a través de ROS2 mediante Ros2-wrapper que se puede encontrar el siguiente repositorio [enlace](https://github.com/Rhobtor/Camera_ZED_ASV). El ros_wrapper_zed permite extraer toda la información que se capta mediante la cámara, cuya información son las siguiente: imágenes RGB, imágenes de profundidad, PointCloud, posicionamiento de la cámara, Odemtria y temperatura. También se encuentra varios parámetros de configuración de la cámara, como servicios de ROS2.

Para la construcción del dockerfile se a utilizado la siguiente imagen, extraída del siguiente repositorio:
```bash
dustynv/ros:humble-ros-base-l4t-r32.7.1
```
Con esta imagen nos permite utilizar la versión de ROS2 Eloquent, junto con el uso de la librería CUDA para la versión seleccionada. La versión seleccionada debe ser la misma que este instalada en la Jetson. LA versión instala del JetPack es la 4.6.1(L4T 32.7.1) , que contiene una versión de Cuda 10.2.

#Nota: Si se cambia la versión del Jetpack se debe cambiar la imagen. Para los JetPack de versión 5, viene integrado la versión de cuda en la propia imagen, sin embargo en las versiones inferiores la versión de cuda viene arraiga a la versión instalada del jetpack. Es decir en las versión de Jetpack 5 podemos cambiar si es compatible con esa versión del Jetpack, la versión de Cuda.

Este siguiente paso es opcional:
```bash
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0
ARG ZED_SDK_PATCH=7
ARG L4T_MAJOR=35
ARG L4T_MINOR=1
ARG ROS2_DIST=humble      # ROS2 distribution
```

Con este paso lo que se realiza es poner las versiones que se quieran utilizar en otras líneas del dockerfile de manera fija. Son como variables de un fichero en un lenguaje de programación. Por ultimo clonamos el repositorio de de ROS-Wrapper-ZED [enlace](https://github.com/stereolabs/zed-ros2-wrapper)

Actualizamos el sistema

```bash
# Disable apt-get warnings
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA || true && \
  apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*
  
ENV TZ=Europe/Paris
ENV ROS_DOMAIN_ID=42
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

# Install the ZED SDK
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO ${ROS2_DIST}

# Install the ZED ROS2 Wrapper
WORKDIR /root/ros2_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
```

Instalamos el ZED SDK, en este caso se utilizo las variables mencionadas anteriormente, si no se utilizan se debe poner las version en las lineas del dockerfile

```bash

# Install the ZED SDK
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev -y && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
  chmod +x ZED_SDK_Linux_JP.run && \
  ./ZED_SDK_Linux_JP.run silent drivers && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so
```



Copiamos los paquetes de Ros2 que utiliza el wrapper, se puede realizar de dos maneras

```bash
# Install missing dependencies
WORKDIR /root/ros2_ws/src
RUN wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
  wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
  wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint && \
  wget https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/${ROBOT_LOCALIZATION_VERSION}.tar.gz -O - | tar -xvz && mv robot_localization-${ROBOT_LOCALIZATION_VERSION} robot-localization && \
  wget https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/${GEOGRAPHIC_INFO_VERSION}.tar.gz -O - | tar -xvz && mv geographic_info-${GEOGRAPHIC_INFO_VERSION} geographic-info && \
  cp -r geographic-info/geographic_msgs/ . && \
  rm -rf geographic-info && \
  git clone https://github.com/ros-drivers/nmea_msgs.git --branch ros2 && \  
  git clone https://github.com/ros/angles.git --branch humble-devel

```

Realizamos un update para ROS eloquent
```bash
# Check that all the dependencies are satisfied
WORKDIR /root/ros2_ws
RUN apt-get update -y || true && rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/*

# Install cython
RUN python3 -m pip install --upgrade cython
```



Realizamos la build de todo el código, para ello en los argumentos del colcon build necesitamos especificar la dirección de las carpeta donde se encuentran los archivos de Cuda. Los argumentos serían los siguientes:

--parallel-workers $(nproc): Esta opción especifica el número de trabajadores a utilizar en paralelo para la compilación. $(nproc) se expande al número de núcleos de CPU disponibles en el sistema.
--symlink-install: Esta opción indica a colcon que instale los paquetes compilados utilizando enlaces simbólicos en lugar de copiar archivos, lo que puede ser más eficiente.
--event-handlers console_direct+: Esta opción especifica cómo se manejarán los eventos de la construcción. En este caso, se está utilizando el manejador console_direct+, que muestra los eventos en la consola en tiempo real.
--base-paths src: Esta opción especifica las rutas base donde se encuentran los paquetes del proyecto.
--cmake-args ...: Estos son argumentos pasados al sistema de construcción CMake durante la compilación. Aquí se están proporcionando varias opciones para configurar la compilación, como el tipo de compilación, las rutas de las bibliotecas de CUDA, las banderas del compilador C++, etc.
--no-warn-unused-cli: Esta opción indica a colcon que no emita advertencias sobre argumentos no utilizados en la línea de comandos.

```bash
WORKDIR /root/ros2_ws

# Build the dependencies and the ZED ROS2 Wrapper
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  colcon build --parallel-workers $(nproc) \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

```

Esta línea copia el archivo ros_entrypoint_jetson.sh del contexto de construcción (es decir, desde el directorio donde se encuentra el Dockerfile) al directorio /sbin/ dentro del contenedor. El archivo ros_entrypoint_jetson.sh es un script que se utilizará como punto de entrada del contenedor para configurar el entorno antes de ejecutar la aplicación principal. el comando chmod para cambiar los permisos del archivo ros_entrypoint.sh dentro del contenedor, dándole permisos de ejecución. Esto es necesario para que el script pueda ser ejecutado como un programa.

```bash
WORKDIR /root/ros2_ws

# Setup environment variables
COPY ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
RUN sudo chmod 755 /sbin/ros_entrypoint.sh
```

Establece el punto de entrada del contenedor. Cuando se inicia el contenedor, se ejecutará el script ros_entrypoint.sh. Esto se hace configurando el punto de entrada del contenedor como el script que hemos copiado y configurado previamente.
```bash
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
```

En cuanto al fichero entrypoint.sh :

Este script es un archivo de shell (ros_entrypoint_jetson.sh) que se ejecutará como punto de entrada del contenedor Docker.

```bash
#!/bin/bash
set -e
```
La línea inicial #!/bin/bash es un shebang que indica al sistema operativo que debe usar el intérprete de comandos bash para ejecutar este script.

La opción -e en el shell, que hace que el script se detenga inmediatamente si ocurre algún error durante su ejecución.
bash

Estas líneas cargan el entorno de ROS 2. La primera línea carga el entorno global de ROS 2 (setup.bash) utilizando la variable de entorno $ROS_DISTRO para determinar la distribución de ROS 2. La segunda línea carga el entorno local del proyecto (local_setup.bash), que puede contener configuraciones específicas para el proyecto en el directorio /root/ros2_ws.
```bash
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash" --
source "/root/ros2_ws/install/local_setup.bash" --
```

Estas líneas imprimen información de bienvenida y detalles sobre la configuración del entorno de ROS 2. Muestra la distribución de ROS ($ROS_DISTRO) y el middleware DDS ($RMW_IMPLEMENTATION).

```bash
# Welcome information
echo "ZED ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION 
echo "---"  
echo 'Available ZED packages:'
```

Estas líneas ejecutan dos comandos de ROS 2 en segundo plano. El primero ejecuta el comando ros2 run para iniciar un nodo llamado static_transform_publisher, que publica transformaciones estáticas entre dos marcos de referencia (base_link y zed2i_camera_center). El segundo ejecuta el comando ros2 launch para iniciar un archivo de lanzamiento (zed2i.launch.py) que probablemente configure y lance nodos relacionados con el sensor ZED.

```bash
# Run your command in the background
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Esta línea ejecuta cualquier comando adicional que se pase al script como argumento. Esto permite al usuario sobreescribir el comportamiento predeterminado del script proporcionando comandos personalizados al ejecutar el contenedor. 

```bash
exec "$@"
```

Antes de realizar la llamada del docker se necesita el siguiente comando
```bash
xhost +si:localuser:root
```

Para realizar la llamada del docker se realiza con el siguiente comando

```bash
docker run -it --gpus all  --privileged --net host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <path folder in hot>:<path folder destination> rhobtor/wrapper_zed
```
-it: Esta opción indica que el contenedor se ejecutará en modo interactivo (interactive) y conectará la terminal del contenedor con la terminal del host (tty).

--gpus all: Esta opción permite que el contenedor acceda a todas las GPU disponibles en el sistema host. Específicamente útil cuando se quiere utilizar el poder de procesamiento de las GPU dentro del contenedor.

--privileged: Esta opción le da al contenedor acceso a todos los dispositivos del sistema host, lo que puede ser necesario para realizar ciertas operaciones de bajo nivel dentro del contenedor.

--net host: Esta opción permite que el contenedor utilice el espacio de red del host, lo que significa que el contenedor compartirá la pila de red del host. Esto puede ser útil para aplicaciones que necesitan acceder directamente a servicios de red del host.

-e DISPLAY: Esta opción establece la variable de entorno DISPLAY dentro del contenedor, que es necesaria para que las aplicaciones gráficas dentro del contenedor se muestren en la pantalla del sistema host.

-v /tmp/.X11-unix:/tmp/.X11-unix: Esta opción monta el socket X11 del sistema host dentro del contenedor en el mismo lugar, permitiendo que las aplicaciones dentro del contenedor se comuniquen con el servidor X11 del host para mostrar sus interfaces gráficas.

-v <path folder in host>:<path folder destination>: Esta opción monta un directorio del sistema host dentro del contenedor en una ubicación específica. Esto puede ser útil para compartir archivos o datos entre el host y el contenedor.

rhobtor/wrapper_zed: Este es el nombre de la imagen del contenedor que se utilizará para crear la instancia del contenedor.

 [Volver](../)   

