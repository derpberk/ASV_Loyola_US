---
ayout: deault
itle: Docker ASV Blue
description: Docker del vehiculo Blueboat
permalink: /blueOS/
---

# Docker BlueOS

Primero debemos concetarnos a la red wifi para poder acceder, tenemos dos opciones WLAN_ASVBLUE (modem 4g) o BlueBoatAP (moden de larga distancia). La contraseña de BlueBoatAP es blueboat.

Para acceder al docker de BluesOS, bastará con ir a la direccion 192.168.1.202 o blueos.local , donde nos cunducirá a la interfaz del docker del BlueOS.

Para activar todas la funciones debemos activar el modo pirata, es un boton en la esquina superior derecha.

![Activar modo pirata](/ASV_Loyola_US/images/pirate_mode.png)

Ahora que tenemos acceso a todos los parametros podemos configurar el BlueOS para nuestra conveniencia.

# Ardupilot firmware

Podemos cambiar la verison de Ardupilot , asi como tambien el tipo de vehiculo que usemos. Ademas, podremos configurar la concexion de los puertos serial, dependiendo de lo que se quiera concetar tanto por USB como por Serial en el hub encima de la raspberry.Tambien podremos realizar un restart del ardupilot si fuera necesario

Actualmente el vehiculo esta en la ultima version estable, con el tipo de vehiculo arduRover

![Puertos serial y version de Ardupilot](/ASV_Loyola_US/images/ardupilot_firmware.png)


# Ardupilot Parameters
Se pueden cambiar los parametros de ardupilot dentro del Docker en la pestaña "Ardupilot Parameters", al igual que en mission planer

![Parametros de Ardupilot](/ASV_Loyola_US/images/parameters_blueos.png)


# BlueOs Version
En la pestaña de BlueOS version se puede modificar la version que esta corriendo el docker, actualmente hay varias versiones ademas de la version de fabrica

Actualmente se encuentra en la version 1.2 del docker de BlueOS

![Versiones del docker BlueOS](/ASV_Loyola_US/images/BlueOS_version.png)

## Importante
Si sale el mensaje de que se ha produzido un error y que se ha vuelto a la version de fabrica, se puede volver a la versión que se estaba utilizando sin problemas.

Para realizarlo , id a la pestaña de BlueOS version y aplicar la version descargaga que se quiera emplear. Esto reiniciara el barco

# MAVLink Injector
En la pestana de Mavlink Endpoints podemos crear nuevos puertos realcionados con mavlink para sus uso. No se reomienda modificar los que ya estan creados desde el inicio
Altualmente se tiene habierto el puerto 5778 para la comunicacion con el navegador y la jetson orin mediante MavLink.

![Configuración de mavlink](/ASV_Loyola_US/images/mavlink_Endpoints.png)


# Mavlink Inspector
Podremos ver los logs de mavlink en la pestaña Mavlink inspector

![Logs de mavlink](/ASV_Loyola_US/images/mavlink_inspector.png)


# Vehicle Setup
Podremos ver en la pestaña Vehicle Setup toda la información del vehiculo, asi como la salida de pwm y la configuracion que tiene

![Parametros del vehiculo](/ASV_Loyola_US/images/vehicle_setup.png)


# Terminal BlueBoat

Existe un terminal en la interfaz de BlueOS, donde podremos realizar las operaciones que se quiera realizar dentro del docker desde un terminal.

![Terminal del docker](/ASV_Loyola_US/images/terminal.png)


# Ethernet + static IP management

Podremos ver la ip del dispositivo que actualmente tiene, en la pestañana superior en el icono de red no aparecera un despliegue  

![Configuración de IP del dispositivo](/ASV_Loyola_US/images/set_ip.png)

Aqui podremos añadir una ip dinamica, un dhcp server o una ip statica. Actualmente se tiene dada la ip de la raspberry 192.168.1.203 mediente el modem 4g (ASV_Blue).  La ip se la asigna mediante una asignacion mediante la mascara de red del dispositivo modem 4G. 


