---
layout: default
title: Docker ASV
description: Docker del vehiculo
permalink: /cmdsdocker/
---

Aqui se encuentran varios comandos de utilidad para los dockers

# Docker ASV

Lanzar el docker_ASV ARM:

- Comando completo (ejemplo)
```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR -e DEBUG="True/False" -e MQTT_ADDR="IP a usar" -e NAVIO_ADDR="IP a usar" --network host bender.us.es:5000/asv_us:arm
```

Lanzar el docker_ASV AMD:
- Comando commpleto (ejemplo)
```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR -e DEBUG="True/False" -e MQTT_ADDR="IP a usar" -e NAVIO_ADDR="IP a usar" --network host bender.us.es:5000/asv_us:amd
```
En el barco se encuentra bajo la llamada de este comando:
- Alias
```bash
run_docker_asv_us
```

con este comando se realiza la llamada del docker con la siguiente configuracion

Barco amarillo:

```bash
docker run -it --rm --device /dev/SONAR --device /dev/SENSOR -e NAVIO_ADDR="192.168.1.203:5678" --network host bender.us.es:5000/asv_us:arm
```

Barco azul:
```bash
docker run -it --rm -e NAVIO_ADDR="192.168.1.203:5778" --network host bender.us.es:5000/asv_us:arm
```
Para lanzarlo sin que ejecute nada, deberemos lanzar el comando del docker junto con alguno de los siguentes comandos 
```bash
--entrypoint bash
```
o
```bash
--entrypoint bin/bash
```
ejemplo:
```bash
docker run -it --rm -e NAVIO_ADDR="192.168.1.203:5778" --entrypoint bin/bash --network host bender.us.es:5000/asv_us:arm
```

# Docker Wrappe_Zed

Para lanzar el docker wrapper_zed en el barco amarillo, se ejecuta el siguente comando:
- Comando

```bash
 docker run -it --gpus all  --privileged --net host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/xavier/CameraFolder:root/CameraFolder bender.us.es:5000/asv_us:zed_wrapper
```
- Para mayor facilidad, este comando de docker esta bajo el alias:

```bash 
run_docker_wrapper
```

Para usar el wrapper con el docker y una pantalla concetada deberemos introducir el siguente comando primero antes de lanzar el docker wrapper_zed:
```bash 
xhost +si:localuser:root
```

# Docker trash detecttion

Para lanzar el docker de deteccion de basura, se realizara para ambos barcos con la siguente llamada del docker:

Barco amarillo:
```bash
docker run -it --network host --runtime nvidia --privileged  bender.us.es:5000/asv_us:trash_xavier_detection
```
Barco azul:
```bash
docker run -it --network host --runtime nvidia --privileged  bender.us.es:5000/asv_us:trash_orin_detection
```

Se ha realizado un mismo alias para la llamada de los dockers:
alias

```bash
run_docker_trash
```

 [Volver](../)   

