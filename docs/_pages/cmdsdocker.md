---
layout: default
title: Docker ASV
description: Docker del vehiculo
permalink: /cmdsdocker/
---

Aqui se encuentran varios comandos de utilidad para los dockers

# Docker ASV

Lanzar el docker_ASV

docker run -it --rm --device /dev/SONAR --device /dev/SENSOR --network bridge syanes/arm:64

Lanzar el docker ASV sin el launch

docker run -it --rm --device /dev/SONAR --device /dev/SENSOR --network bridge syanes/arm:64 /bin/bash


# Docker Wrappe_Zed

Antes de lanzar el docker de wrapper_zed

xhost +si:localuser:root

Lanzar el docker wrapper_zed

docker run -it --gpus all  --privileged --net host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/xavier/CameraFolder:root/CameraFolder rhobtor/wrapper_zed


# Comandos comunes

ver el ID de las imagenes que se estan ejecutando

docker ps 

acceder a la imagen mediante otro terminal
docker exec -it (ID) /bin/bash



