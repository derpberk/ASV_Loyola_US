---
layout: default
title: Hardware
description: Dispositivos que componen el sistema del vehículo
permalink: /hardware/
---

## Elementos del sistema

El sistema se compone de los siguientes elementos:

* Autopiloto Navio2 (Flight Controller Unit)
* Raspberry Pi 4 (Flight Controller Board)
* GPS RTK Emlid M+
* Antena LoRa para comunicación con la estación base Emlid Reach RS+
* Receptor RC con SBUS
* Jetson Nano Xavier NX (Flight Companion Board)
* Cámara estéreo ZED 2
* Router 4G WiFi
* Batería 6S 20000 mAh
* Convertidor DC-DC 6S a 12V
* Power Module 6S
* 2x ESC para motores de propulsión
* Interruptor de seguridad

El conexionado del vehículo con el autopiloto se puede ver en la siguiente imagen:

![Conexionado del vehículo con el autopiloto](/ASV_Loyola_US/images/diagrama_hardware.svg)

---
## Funcionamiento general del sistema
El Navio2 y la Raspberry Pi 4 conforman la unidad de control del vehículo. Dentro de la Raspberry Pi 4 se ejecuta Ardupilot 4.0.0, que permite el guiado del vehículo. La configuración de Ardupilot se puede encontrar en el siguiente [enlace](https://docs.emlid.com/navio2/ardupilot/installation-and-running). A través del rail de pines se controla la velocidad de los motores mediante los circuitos variadores (ESCs). El primer pin del rail de pines está reservado para el receptor del Radio Control en modo SBUS, para la maniobra manual del vehículo.

La RPi4 solo funciona como Autopiloto y no realiza tareas de planificación más que las de atender a las llamadas de alto nivel provenientes de la Jetson Xavier (FCB). La Jetson Xavier es la encargada de realizar la planificación de la misión y de la navegación autónoma. La Jetson Xavier se comunica con la RPi4 a través de una conexión TCP mediante el Router, que establece las IPs de cada elemento de forma fija. La comunicación entre la Jetson Xavier y la RPi4 se realiza mediante el protocolo MAVLink, mientras que la comunicación entre la Jetson Xavier y el Navio2 se realiza mediante el protocolo MAVROS.

El GPS RTK Reach M+ se comunica con Navio2 a través del puerto USB, que también proporciona su alimentación. El Reach M+, aparte de su posición GPS, recibe las correcciones de la estación base mediante la antena LoRa. 

Por último, el router cuenta con una tarjeta SIM que permite la conexión a la red celular de telefonía y la consiguiente comunicación con el servidor central.


---
 [Volver](../)   

 [Siguiente: Preparación del Navio2](/ASV_Loyola_US/navio2/)