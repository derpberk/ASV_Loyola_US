---
layout: default
title: Reach M+
description: Configuración del GPS RTK Reach M+
permalink: /reachm+/
---

## Configuración del GPS RTK Reach M+

Para la configuración del GPS RTK Reach M+, se ha seguido la guía de configuración de Emlid, que se puede encontrar en el siguiente [enlace](https://docs.emlid.com/reachm-plus/common/tutorials/first-setup/).

Lo único que hay que tener en cuenta es lo siguiente:

* Hay que configurar el Reach M+ como *Rover* y no como *Base*.
* En el Rover, las correcciones se transmiten (correction output) a través de USB.
* Hay que seleccionar los baudios correctos (38400) para la comunicación con el Navio2 mediante puerto serie.
* La tasa de envío de mensajes debe ser de 5 Hz.
* El protocolo de salida debe ser ERB.

## Configuración de parámetros de Ardupilot

Para la configuración de los parámetros de Ardupilot, se ha seguido la guía de configuración de Emlid, que se puede encontrar en el siguiente [enlace](https://docs.emlid.com/reachm-plus/common/tutorials/first-setup/).

Los parámetros que se han configurado son los siguientes:

* `GPS_TYPE2`: 13 (ERB)
* `SERIAL4_BAUD `: 38 (38400 baudios)
* `GPS_AUTO_SWITCH `: 1 (Enabled)
* `GPS_INJECT_TO  `: 1 (Secondary)

 [Volver](/ASV_Loyola_US/udevrule/) 

 [Siguiente: Configuración Sonda AML-3 XC](/ASV_Loyola_US/sensoresaml/)