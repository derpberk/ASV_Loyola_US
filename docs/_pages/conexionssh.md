---
layout: default
title: Conexión SSH
description: Cómo conectarnos por SSH al vehículo
permalink: /conexionssh/
---

## Conexión SSH con Navio2 / Raspberry Pi

Para conectarnos con Navio2, tenemos que estar conectados en la misma red local que Navio2. Por defecto, Navio2 se conecta
al Router del vehículo con la IO `192.168.1.204`. Existe un hostname que podemos usar: `navio.local`. Para iniciar una sesión de SSH:

```bash
ssh pi@navio.local # También vale ssh pi@192.168.1.204
```

La contraseña por defecto es `raspberry`.


## Conexión SSH con Xavier NX

Para conectarnos con Navio2, tenemos que estar conectados en la misma red local que Navio2. Por defecto, Navio2 se conecta
al Router del vehículo con la IO `192.168.1.203`. Existe un hostname que podemos usar: `xavier.local`. Para iniciar una sesión de SSH:

```bash
ssh jetson@xavier.local # También vale ssh jetson@192.168.1.203
```

La contraseña por defecto es `xavier`.

