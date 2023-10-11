---
layout: default
title: Reglas de asociación USB	
description: Cómo crear una regla udev
permalink: /udevrule/
---

## ¿Para qué sirve una regla udev?

Una regla udev sirve para crear un nombre de dispositivo específico para un dispositivo USB. Esto es útil cuando se conectan varios dispositivos USB y se quiere que cada uno tenga un nombre específico. En nuestro caso, queremos que el Reach M+ tenga el nombre `/dev/REACH` para que el Navio2 lo reconozca, y no cualquier nombre como `/dev/ttyACM0` o `/dev/ttyACM1`, según el puerto USB al que se conecte y el orden de conexión.

## Creación de una regla udev

Para crear una regla udev, se crea un archivo en el directorio `/etc/udev/rules.d/` con el nombre `99-reach.rules` (el nombre del archivo puede ser cualquiera, pero se recomienda que empiece por un número alto para que se ejecute al final). El contenido del archivo es el siguiente:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="REACH"
```

Para obtener los valores de `idVendor` e `idProduct`, se ejecuta el siguiente comando **conectando el dispositivo Reach M+ al Navio2**:

```bash
pi@navio: ~ $ lsusb

Bus 001 Device 004: ID 1546:01a8 U-Blox AG [u-blox 8]
```

El valor de `idVendor` es `1546` y el valor de `idProduct` es `01a8`. Con esto, ya se puede crear la regla udev.

## Comprobación de la regla udev

Para comprobar que la regla udev se ha creado correctamente, se ejecuta el siguiente comando:

```bash
pi@navio: ~ $ ls -l /dev/REACH

lrwxrwxrwx 1 root root 7 May  4 11:50 /dev/REACH -> ttyACM0
```

Si el comando devuelve el nombre del dispositivo USB, la regla udev se ha creado correctamente. Si no, se debe reiniciar el Navio2 y volver a ejecutar el comando.

 [Volver](../)   

 [Siguiente: Configuración del Reach M+](/ASV_Loyola_US/reachm+/)