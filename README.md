# Navegacion
## Organización del repositorio
Este respositorio contiene 2 carpetas: 
- navegacion_autonoma: Contiene los archivos de python que permiten la comunicación con 
la kobuki real y la simulación. Los archivos
principales son: 
    - rosanautica: Es una clase meramente logica, que se encarga de generar la función potencial dada una matriz con     1 y 0, donde 1 representa que existe un obstaculo en dicha posición. 
    - mover: Es un ActionServer, que permite emplear a rosa nautica para obtener la planeación de movimientos del robot, ademas de leer los valores del odometro y finalmente enviar las isntrucciones a la kobuki. Algo que cabe mencionar, es que para una futura implementación este server, debería conectarse con alguna fuente de información que provea la posición actual del robot. 
- navegacion_interfaces: Unicamente contiene la definción de los tipos de datos necesarios para usar navegación. 
## Instrucciones para correr el proyecto
Primero es necesario instalar las dependencias adicionales:
```
rosdep install --from-paths src -y --ignore-src
```
Luego hay que hacer un colcon build: 
```
colcon build --symlink-install
```
Luego es importante hacer un source para tener las definiciones de los tipos de mensajes que se encuentran en navegacion_interfaces: 
```
source install/setup.bash
```
Luego si bien se puede correr el nodo como:
```
ros2 run navegacion_autonoma mover

```
Se sugiere unicamente entrar a la carpeta 
```
cd navegacion_autonoma/navegacion_autonoma

```
Y ejecutar el script como un script de python usual, esto para no tener que recompilar todo, cada vez que se haga un cambio en el codigo: 
```
python3 mover.py
```
Finalmente para poder llamar al servicio, se 
realiza con el siguiente comando en donde indicamos hacia donde se va a mover el robot.
```
ros2 action send_goal --feedback mover navegacion_interfaces/action/Navegacion "{x: 5, y: 1}"
```
