---
layout: default
title: Hardware
description: Dispositivos que componen el sistema del vehículo
permalink: /hardware/
---

## Hardware del sistema

## Autopiloto

El autopiloto es el sistema encargado de controlar el vehículo de forma autónoma. El autopiloto que incorpora el vehículo es un Navio2 de la empresa Emlid. El Navio2 es un hat para una Raspberry Pi 3b o 4 que incorpora un sistema operativo Raspbian Jesse. El Navio2 incorpora los siguientes sensores:

- IMU MPU9250
- Barómetro MS5611
- Magnetómetro HMC5883L
- GPS u-blox NEO-M8N
- ADC ADS1115
- RC I/O

El sistema de autopiloto nos permite utilizar el software de código abierto ArduPilot. ArduPilot es un autopiloto de código abierto para vehículos no tripulados. ArduPilot soporta múltiples plataformas, incluyendo vehículos aéreos, terrestres y acuáticos. ArduPilot es compatible con múltiples sistemas operativos, incluyendo Linux, Windows y Mac OSX. ArduPilot es un proyecto de código abierto que se encuentra bajo la licencia GPLv3.

El conexionado del vehículo con el autopiloto se puede ver en la siguiente imagen:

![Conexionado del vehículo con el autopiloto](../assets/images/Conexionado_autopiloto.png)


[Home](../)
