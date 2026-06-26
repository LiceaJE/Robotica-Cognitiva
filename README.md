# Robotica-Cognitiva
Proyecto final de Robótica cognitiva

Ambas versiones del proyecto se encuentran en este repositorio, el mundo de webots (Gauss2.wbt) y el supervisor se usan en cualquiera de las dos versiones del proyecto, sea el modelo neuronal o la Q-table.

## Modelo Q-Table
Para usarlo simplemente asignar los controladores a cada robot, el controlador del supervisor al supervisor y controlador de la carpeta al e-puck

## Modelo Neuronal
Para que funcione primero se debe entrenar el modelo usando el ipynb y el .csv extraido usando los controladores babbling. Después con los pesos guardados de la red correr el server_cerebro_2.py y con el controlador e-puck25.c asignado en webots correr la simulación.
