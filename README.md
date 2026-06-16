
# Proyecto: Plantilla de Navegación (ROS2)

Este paquete proporciona una base educativa para trabajar con odometría, Lidar y control de movimiento en robots holonómicos o diferenciales.

**Estructura principal**
- `proyecto/` : Código fuente del paquete.
	- `navigation_node.py` : Nodo principal que expone funciones de utilidad para estudiantes (leer Lidar, rotar, mover relativo, cargar escenas, menú interactivo).
	- `offset_node.py` : Nodo que transforma `/odom` en `/odom_offset` para emular el marco global de Gazebo cuando el robot real arranca con odometría en cero.
	- `logic/lidar.py` : Funciones de ayuda para extraer distancias y rangos del mensaje `LaserScan`.
	- `logic/movement.py` : Algoritmos para calcular velocidades angulares/lineales y perfiles de movimiento relativos.

- `data/` : Archivos de escena (`Escena-Problema1.txt` ... `Escena-Problema6.txt`) usados por el nodo para pruebas y visualización textual.
- `resource/` : Recursos de empaquetado.
- `test/` : Tests básicos y comprobaciones de estilo.

**Características importantes**
- Interfaz de consola no bloqueante: `navigation_node.py` inicia un hilo con un menú interactivo para probar funciones durante la ejecución.
- Wrappers didácticos: Métodos como `leer_distancia_en_angulo`, `leer_distancias_en_rango`, `rotar_relativo`, `mover_relativo` están pensados para que estudiantes llamen fácilmente desde el menú o integren en ejercicios.
- Manejo de escenas: `cargar_escena(numero)` lee el archivo `data/Escena-Problema{numero}.txt` y guarda su contenido en `self.texto_escena`.
- Compatibilidad simulación / robot real: `offset_node.py` permite publicar una odometría transformada en `/odom_offset` para que el planeador trabaje con las mismas coordenadas globales definidas en los archivos de escena.

**Cómo ejecutar (entorno ROS2)**
1. Fuente del workspace (desde la raíz del workspace):

```bash
colcon build --symlink-install
source install/setup.bash
```

2. Ejecutar el nodo (desde un terminal con ROS2 configurado):

```bash
ros2 run proyecto navigation
```

3. Ejecutar el nodo de offset si se va a trabajar con robot real:

```bash
ros2 run proyecto offset
```

Nota: El paquete asume tópicos `odom`, `scan_raw` y `cmd_vel` disponibles en el bus ROS2 (simulador Gazebo, robot real o nodo de prueba).

## Nodo de offset

El nodo `offset_node.py` escucha la odometría en `/odom` y publica una versión transformada en `/odom_offset`.

Esto se usa principalmente en robot real, porque el robot suele arrancar con odometría local en `(0, 0, 0)`, mientras que en Gazebo cada escena ya está definida en un sistema de coordenadas global. El objetivo del nodo es hacer que la odometría del robot real se comporte como la de la simulación, de forma que la pose inicial publicada coincida con el `q0` de la escena seleccionada.

### Qué hace

- Opción `0`: no aplica ninguna transformación; solo republíca `/odom` en `/odom_offset`.
- Opción `3`: hace que la pose inicial del robot corresponda a `q0 = (0.75, 0.75, 0°)`.
- Opción `5`: hace que la pose inicial del robot corresponda a `q0 = (3.25, 0.75, 180°)`.
- Opción `7`: hace que la pose inicial del robot corresponda a `q0 = (0.25, 4.25, 90°)`.
- Opción `8`: hace que la pose inicial del robot corresponda a `q0 = (0.75, 0.75, 90°)`.

En las opciones distintas de `0`, el nodo captura la primera pose recibida desde `/odom` y la usa como origen local del robot real. A partir de ahí transforma posición y orientación para publicarlas en el marco global esperado por la escena.

### Cómo ejecutarlo

Modo sin transformación:

```bash
ros2 run proyecto offset --ros-args -p option:=0
```

Escena 3:

```bash
ros2 run proyecto offset --ros-args -p option:=3
```

Escena 5:

```bash
ros2 run proyecto offset --ros-args -p option:=5
```

Escena 7:

```bash
ros2 run proyecto offset --ros-args -p option:=7
```

Escena 8:

```bash
ros2 run proyecto offset --ros-args -p option:=8
```

Si no se pasa el parámetro `option`, el nodo usa por defecto la opción `3`.

### Recomendación de uso

- En simulación Gazebo normalmente no hace falta este nodo si ya se está usando la odometría global esperada por la escena.
- En robot real, conviene correr `offset` antes de `navigation`, y configurar `navigation_node.py` para consumir `/odom_offset` cuando se quiera reproducir el comportamiento de Gazebo.

**Editar y extender**
- Para cambiar la lógica del Lidar, revise `proyecto/logic/lidar.py`.
- Para ajustar perfiles de movimiento o tolerancias, edite `proyecto/logic/movement.py`.
- Para añadir nuevas escenas, coloque archivos `Escena-ProblemaX.txt` en la carpeta `data/`.
