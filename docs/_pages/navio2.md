---
layout: default
title: Navio 2
description: Autopiloto del vehículo
permalink: /navio2/
---

## Preparación de la tarjeta SD

La instalación de Navio2 está recogida en el siguiente [enlace](https://docs.emlid.com/navio2/configuring-raspberry-pi). En este caso, se ha instalado la imagen de Navio2 para Raspberry Pi 4.

El primer paso es instalar la imagen de Emlid en la tarjeta SD. Para ello, se descarga la imagen de Navio2 para Raspberry Pi 4 desde el siguiente [enlace](https://docs.emlid.com/navio2/common/dev/flash-sd-card/). Una vez descargada, se descomprime el archivo y se graba la imagen en la tarjeta SD con el software [Etcher](https://www.balena.io/etcher/).

## Instalación de Ardupilot (ArduRover)

El barco se manejará como un robot tipo Rover debido a sus particularidades de movimiento. Para acceder al Navio2, hay que conectarnos a la Raspberry Pi 4 mediante Ethernet o a la misma red WiFi a la que está conectado el Navio2. Para acceder a Navio2 es posible usar un *hostname* o una dirección IP. El *hostname* por defecto es `navio.local`. La dirección IP por defecto en el router es `192.168.1.203`. Para acceder a Navio2, se puede usar el siguiente comando:

```bash
ssh pi@navio.local
```

La contraseña por defecto es `raspberrypi`.

Una vez dentro de Navio2, seleccionamos el tipo de vehículo (ArduRover):

```bash
pi@navio: ~ $ sudo emlidtool ardupilot
```

Es necesario especificar la configuración de comunicación del sistema. Para ello, se ejecuta el siguiente comando:

```bash
pi@navio: ~ $ sudo nano /etc/default/ardurover
```

Este archivo de configuración define la comunicación del Navio2 con 1) el Reach M+, 2) Mission Planner, 3) la Jetson Xavier. Se crearán tantos puertos de telemetría `TELEM1`, ..., `TELEM7`, como dispositivos se quieran conectar. Dependiendo del tipo de conexión, se pondrá el tipo de conexión y el puerto correspondiente. Los tipos de conexión son:

* `-A` para conexiones de telemetría a través de WiFi o Ethernet (siempre 115200 baudios)
* `-C` para conexiones de telemetría (como las radios SiK y similares a 57600 baudios).
* `-E` para conexiones con GPS secundario (38400 baudios)

El archivo final quedará así:

```bash	
TELEM1="-A tcp:192.168.1.204:5678"  # Con esto, enlazamos el Navio2 con la Jetson Xavier
TELEM2="-C /dev/ttyAMA0" # Esto es para conectar el Navio2 con una radio SiK
TELEM3="-E /dev/REACH" # Esto es para conectar el Navio2 con el Reach M+. Ojo que el puerto USB /dev/REACH puede cambiar y llamarse /dev/ttyACM0 o /dev/ttyACM1. Para enlazar el Reach M+ con el nombre /dev/REACH, se debe crear una regla udev.

ARDUPILOT_OPTS="$TELEM1 $TELEM2 $TELEM3"
```

Una vez configurado el archivo, se guarda y se reinicia el servicio de Ardupilot:

```bash
pi@navio: ~ $ sudo systemctl daemon-reload
```

Para iniciar el servicio de Ardupilot al arrancar el Navio2, se ejecuta el siguiente comando:

```bash
pi@navio: ~ $ sudo systemctl enable ardurover
```

 [Volver](/ASV_Loyola_US/hardware/)   

 [Siguiente: Creación de una UDEV Rule](/ASV_Loyola_US/udevrule/)