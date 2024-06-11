---
layout: default
title: Docker ASV
description: Docker del vehiculo
permalink: /cmdsdocker/
---

Aqui se encuentran varios comandos de utilidad para los dockers

# Docker ASV

Lanzar el docker_ASV ARM:
- Comando
```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR -e DEBUG="True" -e MQTT_ADDR="IP a usar" -e NAVIO_ADDR="IP a usar" --network host bender.us.es:5000/asv_us:arm
```

Lanzar el docker_ASV ARM:
- Comando
```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR -e DEBUG="True" -e MQTT_ADDR="IP a usar" -e NAVIO_ADDR="IP a usar" --network host bender.us.es:5000/asv_us:amd
```

- Alias
```bash
run_docker_asv_us
```
# Importante!!!!
Todas las flags "-e " son opcionales, si no se quiere usar no hace falta escribirlas, el codigo se lanzará con la configuración con la que se haya creado el docker 

La flag DEBUG es True o False dependiendo si queremos el codigo en modo debug o normal

Si queremos concetarnos al navio2 debemos usar la flag como:
```bash
 -e NAVIO_ADDR="192.168.1.203:5678" 
 ```

si es con el SITL poner
```bash
-e NAVIO_ADDR="127.0.0.1:5788"
```
o 
```bash
-e NAVIO_ADDR="(vuestra ip):5788"
```
Para lanzar el docker con el con bash deberemos añadir el siguiente flag:

```bash
--entrypoint bash
```
o
```bash
--entrypoint bin/bash
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

