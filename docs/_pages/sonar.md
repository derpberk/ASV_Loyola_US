---
layout: default
title: Sonar
description: Sonar Blue Robotics
permalink: /sonar/
---

## Sonar Blue Robotic

La sonda Ping2 es una ecosonda de un solo haz que mide distancias de hasta 100 metros (328 pies) bajo el agua. Un ancho de haz de 25 grados, una profundidad nominal de 300 metros (984 pies) y una interfaz de software de código abierto lo convierten en una poderosa herramienta para la robótica marina.

Para utilizar el Sonar, Blue Robotic nos ofrece la libreria Ping. Para la instalacion de la libreria Ping podemos utilizar el siguente comando:

```bash
pip install --user bluerobotics-ping --upgrade
```

Podemos instarlo desde el codigo con el siguente comando:

```bash
git clone --single-branch --branch deployment https://github.com/bluerobotics/ping-python.git
cd ping-python
python setup.py install --user
```

En el fichero python importamos la libreria y la utilizamos de la siguente manera:

```bash
from brping import Ping1D

sonar.device = Ping1D()

```

 [Volver](../)   

 [Siguiente: Configuración del Reach M+](/ASV_Loyola_US/docker_asv/)
