---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---



En esta página se muestran los mapas configurados para los ASV, las zonas azules indican las zonas por las que el dron puede desplazarse cuando el [planificador](./src/asv_loyola_us/asv_loyola_us/planner_node.html) esta activado

<H3> Alamillo95x216 <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/mapas/Alamillo95x216plantilla.csv" style="float:right;text-align:right;">mapbit</a></H3>




<p style="text-align:center;"><img src="./miscelaneous/Alamillo.png" width="80%"/></p>
<p style="text-align:center;"><img src="./miscelaneous/Alamillo95x216gridmask.png" width="100%"/></p>



<H3> Loyola121x239 <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/mapas/Loyola121x239plantilla.csv" style="float:right;text-align:right;">mapbit</a></H3>




<p style="text-align:center;"><img src="./miscelaneous/Loyola.png" alt="drawing" width="60%"/></p>
<p style="text-align:center;"><img src="./miscelaneous/Loyola121x239gridmask.png" alt="drawing" width="100%"/></p>


<H3> Creacion de mapas <a href="https://github.com/AloePacci/ASV_Loyola_US/blob/main/other%20code%20for%20development/create_grid.sh" style="float:right;text-align:right;">code</a></H3>

Para la creación de mapas se hace uso del script create grid. Para ello se hace uso de 3 datos:
- Coordenadas GPS de la esquina noroeste del mapa
- Coordenadas GPS de la esquina sureste del mapa
- Tamaño en metros de la celda

 El proceso se realiza en dos fases, una primera fase crea los archivos necesarios para crear la plantilla. Haciendo uso de cualquier editor de imágenes, se pintarán de negro aquellas zonas que consideremos obstáculos.

 Una vez creado, se ejecutará el mismo script e indicará el archivo de la plantilla, tras el que se creará la plantilla de 1 y 0

 Los archivos necesarios son:
 - \name\plantilla.csv
 - \name\grid.csv