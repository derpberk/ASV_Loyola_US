---
layout: default
title: Sensores AML
description: Sonda de sensores AML Oceanographic
permalink: /sensoresaml/
---

## Sonda AML-3 XC

La sonda de sensores de calidad del agua AML-3 XC cuenta con 3 sensores:

* Conductividad (mS/m^2)
* Temperatura (ºC)
* Turbidez (NTU)
* pH (unidades de pH)

Las características de los sensores son los siguientes:

![Características de los sensores](/ASV_Loyola_US/images/sensor_chars.png)

La sonda tiene una batería interna que se puede cargar con un USB-C normal. Una vez se enciende la sonda, por defecto, la sonda comienza a tomar medidas una vez detecta que está sumergida.
Es posible comunicarse con la sonda utilizando el puerto RS232. El protocolo de comunicación se basa en una serie de comandos que se envían a la sonda y que devuelven una respuesta. La sonda tiene un *baud rate* de 115200 baudios (valor predeterminado).

Los siguientes comandos son útiles y se pueden utilizar para comunicarse con la sonda:

* `scan`: La sonda comienza a mandar medidas con formato AMLx de forma ininterrumpida con una frecuencia determinada.
* `set sample [RATE] secons`: Establece la frecuencia de muestreo de la sonda en segundos en *RATE* sampleos por segundo.
* `mscan`: La sonda envía SOLO una medida con formato AMLx.

El formato AMLx es tal y como sigue:

`msg<MsgNumber>{mux[meta=time,<UnixEpochTime>,s][data=uv,<status>],port<PortNumbr>[data=<ParameterName>,<Value>,<ParameterUnits>][rawi=<RawParameterName>,<RawValue>,<RawUnits>]}`

Se puede obtener el dato de cada sensor simplemente buscando el nombre del sensor en el mensaje. Por ejemplo, para obtener la conductividad, se puede buscar `cond` en el mensaje. Estos
son los nombres de los sensores en cada mensaje:

* Conductivity: `Cond`
* Temperature: `TempCT`
* pH: `pH`
* Turbidity: `Turbidity`



 [Volver](/ASV_Loyola_US/reachm+/)

 [Siguiente: Configuración Sonar Blue Robotics](/ASV_Loyola_US/sonar/)
