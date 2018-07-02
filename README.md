# minesweepers2018
Repo de código para el robot del equipo ART para la competencia Minesweepers 2018 Bolivia

## Paquetes y nodos
En este repositorio se encuentran 4 carpetas correspondientes a 4 módulos para el robot.
  - adc_ros: esta carpeta contiene el código fuente para un nodo de lectura de los sensores de minas implementado en una placa arduino utilizando la librería rosserial_arduino y el nodo rosserial_python para publicar los mensajes en tópicos de ROS.
  - robochoto: Este paquete de ROS contiene el codigo fuente de los nodos y los archivos launch para las funciones de teleoperacion y monitoreo remoto del robot.
  - rqt_monitor_robochoto: Este paquete incluye un plugin de RQT para el monitoreo en tiempo real de los sensores de minas del robot.
  - rqt_robochoto: Este paquete contiene el código fuente del plugin de RQT para el mapeo y reporte del campo minado de la competencia.

