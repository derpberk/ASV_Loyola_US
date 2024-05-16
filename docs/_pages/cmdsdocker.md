---
layout: default
title: Docker ASV
description: Docker del vehiculo
permalink: /cmdsdocker/
---

Aqui se encuentran varios comandos de utilidad para los dockers

# Docker ASV

Lanzar el docker_ASV:
- Comando
```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR --network host bender.us.es:5000/asv_us:latest
```
- Alias
```bash
run_docker_asv_us
```


# Docker Wrappe_Zed

Antes de lanzar el docker de wrapper_zed

xhost +si:localuser:root

Lanzar el docker wrapper_zed con :
- Comando

```bash
 docker run -it --gpus all  --privileged --net host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/xavier/CameraFolder:root/CameraFolder bender.us.es:5000/asv_us:zed_wrapper
```
- alias

```bash 
run_docker_wrapper
```

# Comandos comunes

ver el ID de las imagenes que se estan ejecutando

docker ps 

acceder a la imagen mediante otro terminal
docker exec -it (ID) /bin/bash



 [Volver](../)   

