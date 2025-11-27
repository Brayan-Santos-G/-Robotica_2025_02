# Laboratorio_04
* Sergio Avellaneda Piñeros  
* David Santiago Cuellar Lopez  
* Brayan Yesid Santos Gonzalez  

## Documentación

## Metodología, Resultados, Análisis y Conclusiones

### Metodología

El desarrollo del laboratorio se realizó en dos etapas principales: control manual y dibujo automático de letras.

### 1. Control manual de la tortuga

1. Se instaló y configuró ROS 2 Humble junto con el simulador Turtlesim.  
2. En una terminal se ejecutó el simulador usando:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

3. Se creó el paquete encargado del control manual:

   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_turtle_controller
   ```

4. Dentro del paquete se implementó el archivo `move_turtle.py`, que incluye:
   - Importación del mensaje `Twist`.
   - Función para lectura de teclado en modo raw.
   - La clase `TurtleMover`, con un temporizador que ejecuta `control_loop()`.
   - Publicación de mensajes al tópico `/turtle1/cmd_vel` según las teclas presionadas.

5. Se compiló el paquete:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

6. Se ejecutó el nodo:

   ```bash
   ros2 run my_turtle_controller move_turtle
   ```

---

### 2. Dibujo automático de letras personalizadas

1. Se creó un segundo script Python que implementa un nodo encargado de dibujar letras mediante trayectorias programadas.
2. Se desarrolló la clase `TurtleController`, la cual:
   - Publica comandos `Twist` para mover la tortuga.
   - Se suscribe al tópico `/turtle1/pose`.
   - Utiliza el servicio `/turtle1/teleport_absolute` para regresar al punto inicial.
   - Maneja la lectura del teclado en un hilo independiente.
   - Incluye funciones auxiliares para movimientos lineales, giros y teletransportación.
3. Se asignaron las letras a los integrantes del grupo:
   - **Sergio Avellaneda Piñeros:** S, A, P  
   - **David Santiago Cuéllar López:** D, C  
   - **Brayan Yesid Santos González:** B, Y, G, L  
4. Cada letra se construyó combinando desplazamientos rectos, giros y arcos generados por velocidad angular + lineal.
5. Una vez completada cada letra, la tortuga vuelve automáticamente a su pose inicial usando teletransportación para mantener orden y simetría en el dibujo.

---

## Resultados

1
<img width="1322" height="636" alt="image" src="https://github.com/user-attachments/assets/b338ace6-9d5a-4234-9a45-81bd99236fc5" />

2

<img width="1251" height="622" alt="image" src="https://github.com/user-attachments/assets/c61cbc42-3b2c-49a3-9f64-85b66307eda4" />

3

<img width="1269" height="630" alt="image" src="https://github.com/user-attachments/assets/c31c1355-2dd6-4b9e-9223-35db81ba6957" />
---

## Análisis

### 1. Comunicación por tópicos
El uso del mensaje `Twist` permitió observar:
- La asincronía entre nodos productores y consumidores.  
- La necesidad de publicar constantemente mientras se desea mantener velocidad.  
- El funcionamiento distribuido del modelo de comunicación de ROS 2.

### 2. Servicios
El servicio `TeleportAbsolute` evidenció:
- Cómo se diferencian acciones puntuales (servicios) de flujos continuos (tópicos).  
- El uso de `call_async()` para ejecutar servicios sin bloquear el nodo.  
- La verificación del estado de la llamada mediante `future.done()`.

### 3. Concurrencia
La lectura del teclado en un hilo independiente permitió:
- Evitar bloqueos en `rclpy.spin()`.  
- Mantener interacción en tiempo real con la simulación.  
- Ejecutar dibujo y lectura de entrada sin interferencias.

### 4. Cinemática de movimiento
Las figuras fueron construidas combinando:
- Desplazamientos rectos con velocidad lineal constante.  
- Giros con velocidad angular constante.  
- Arcos circulares mediante velocidad lineal + velocidad angular.  

Esto permitió comprender mejor la cinemática de un robot diferencial y su relación con trayectorias complejas.

---

## Conclusiones

- Se lograron comprender los conceptos fundamentales del sistema ROS 2: nodos, tópicos, mensajes, servicios, temporización y concurrencia.
- Se implementó correctamente un nodo para controlar la tortuga manualmente y un nodo avanzado para dibujar letras mediante trayectorias definidas.
- El laboratorio permitió reforzar la estructura de comunicación distribuida de ROS 2, especialmente la interacción entre publicadores y servicios.
- Todas las letras programadas por los integrantes del grupo se dibujaron correctamente, demostrando una adecuada manipulación de movimiento diferencial.
- El uso de teletransportación proporcionó limpieza y organización en los resultados visuales.
- El sistema final operó de manera continua y estable, integrando publicación de comandos, lectura de pose, teletransportación y entrada por teclado en tiempo real.



## Diagrama de flujo
<img width="221" height="509" alt="image" src="https://github.com/user-attachments/assets/f79bc965-d0a2-4c94-aab9-0bbd01ff7bcb" />
<img width="1090" height="414" alt="image" src="https://github.com/user-attachments/assets/0981b282-3152-4842-8767-e02fa083d8ea" />
<img width="1090" height="401" alt="image" src="https://github.com/user-attachments/assets/1d310712-ed65-4d40-8d79-710cadb838ff" />
<img width="1126" height="412" alt="image" src="https://github.com/user-attachments/assets/bfe601c9-2e80-40b3-856f-72d342e776f0" />


## Codigo

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

import sys
import tty
import termios
import threading
import time
import math

"""
Nodo de ROS 2 para controlar la tortuga de turtlesim desde el teclado.
Permite dibujar letras específicas (S, A, P, B, Y, G, D, C, L) usando únicamente
velocidad lineal y angular. Después de cada letra, la tortuga vuelve al punto
inicial mediante el servicio de teleport.
"""

class TurtleController(Node):
    """Nodo principal que publica cmd_vel, escucha la pose y procesa el teclado."""

    def __init__(self):
        super().__init__('turtle_controller')

        # Publicador de velocidad para la tortuga
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Pose actual de la tortuga (se actualiza por callback)
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Cliente para el servicio de teleport
        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute'
        )

        # Hilo independiente para leer teclas sin bloquear rclpy.spin()
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def pose_callback(self, msg: Pose):
        """Guarda la pose actual de la tortuga."""
        self.current_pose = msg

    def get_key(self) -> str:
        """
        Lee una sola tecla en modo raw (sin Enter).
        Bloquea hasta que el usuario presione algo.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_listener(self):
        """
        Hilo que escucha continuamente el teclado
        para ejecutar una letra según la tecla presionada.
        """
        while rclpy.ok():
            key = self.get_key()
            if key.lower() == 's':
                self.get_logger().info("Dibujando letra S")
                self.draw_S()
            elif key.lower() == 'a':
                self.get_logger().info("Dibujando letra A")
                self.draw_A()
            elif key.lower() == 'p':
                self.get_logger().info("Dibujando letra P")
                self.draw_P()
            elif key.lower() == 'b':
                self.get_logger().info("Dibujando letra B")
                self.draw_B()
            elif key.lower() == 'y':
                self.get_logger().info("Dibujando letra Y")
                self.draw_Y()
            elif key.lower() == 'g':
                self.get_logger().info("Dibujando letra G")
                self.draw_G()
            elif key.lower() == 'd':
                self.get_logger().info("Dibujando letra D")
                self.draw_D()
            elif key.lower() == 'c':
                self.get_logger().info("Dibujando letra C")
                self.draw_C()
            elif key.lower() == 'l':
                self.get_logger().info("Dibujando letra L")
                self.draw_L()
            elif key == '\x03':  # Ctrl+C
                break

    def teleport(self, x, y, theta):
        """Usa el servicio teleport_absolute para mover la tortuga a una pose exacta."""
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta

        future = self.teleport_client.call_async(req)
        start_time = time.time()
        while not future.done() and (time.time() - start_time < 2.0):
            time.sleep(0.05)

        if future.done():
            try:
                future.result()
            except Exception as e:
                self.get_logger().error(f"Error al teleportar: {e}")
        else:
            self.get_logger().error("Teleport no completado a tiempo")

    def move_distance(self, distance, speed=2.0):
        """Mueve la tortuga una distancia usando velocidad lineal constante."""
        duration = abs(distance) / speed
        msg = Twist()
        msg.linear.x = speed if distance >= 0 else -speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        # detener
        self.publisher_.publish(Twist())
        time.sleep(0.08)

    def turn_angle(self, angle, angular_speed=1.0):
        """Gira la tortuga un ángulo (rad) usando velocidad angular constante."""
        duration = abs(angle) / angular_speed
        msg = Twist()
        msg.angular.z = angular_speed if angle >= 0 else -angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        # detener giro
        self.publisher_.publish(Twist())
        time.sleep(0.08)

    def save_initial_pose(self):
        """Devuelve la pose actual para restaurarla después de dibujar."""
        if self.current_pose is None:
            self.get_logger().error("Pose no disponible")
            return None
        return (self.current_pose.x, self.current_pose.y, self.current_pose.theta)

    # ───── Letras ─────────────────────────────────────────────────────────────

    def draw_S(self):
        """Dibuja la letra S."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        radius = 1.5 / 2
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        # inicio del trazo
        self.turn_angle(math.radians(180))
        self.move_distance(1.5)
        self.turn_angle(math.radians(180))

        # semicírculo superior
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        duration = math.pi / angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())

        # semicírculo inferior
        msg.angular.z = -angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)

    def draw_A(self):
        """Dibuja la letra A."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0

        self.turn_angle(math.radians(-120))
        self.move_distance(math.sqrt(13))
        self.teleport(x, y, theta)

        self.turn_angle(math.radians(-90))
        self.move_distance(h)

        self.turn_angle(math.radians(-120))
        self.move_distance(1.5)

        self.teleport(x, y, theta)

    def draw_P(self):
        """Dibuja la letra P."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0
        radius = h / 4
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        self.turn_angle(math.radians(-90))
        self.move_distance(h)
        self.teleport(x, y, theta)

        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        duration = math.pi / angular_speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)

    def draw_B(self):
        """Dibuja la letra B."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0
        radius = h / 4
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        self.turn_angle(math.radians(90))
        self.move_distance(h)

        # semicírculo superior
        self.turn_angle(math.radians(-90))
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        duration = math.pi / angular_speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        time.sleep(0.05)

        # semicírculo inferior
        self.turn_angle(math.radians(180))
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)

    def draw_Y(self):
        """Dibuja la letra Y."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3

        self.turn_angle(math.radians(-90))
        self.move_distance(h)

        self.turn_angle(math.radians(180))
        self.move_distance(h / 2)

        self.turn_angle(math.radians(40))
        self.move_distance(h / 2)

        self.teleport(x, y, theta)

    def draw_G(self):
        """Dibuja la letra G."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        radius = 1.5
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        self.move_distance(1.5)
        self.turn_angle(math.radians(180))

        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        duration = (3 * math.pi / 2) / angular_speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        time.sleep(0.05)

        # trazo interno
        self.turn_angle(math.radians(90))
        self.move_distance(radius)

        self.teleport(x, y, theta)

    def draw_D(self):
        """Dibuja la letra D."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0
        radius = h / 2
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        self.turn_angle(math.radians(90))
        self.move_distance(h)

        self.turn_angle(math.radians(-90))
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        duration = math.pi / angular_speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)

    def draw_C(self):
        """Dibuja la letra C."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        radius = 1.5
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        self.turn_angle(math.radians(180))

        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        duration = math.pi / angular_speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)

    def draw_L(self):
        """Dibuja la letra L."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0

        self.turn_angle(math.radians(-90))
        self.move_distance(h)

        self.turn_angle(math.radians(90))
        self.move_distance(h / 2)

        self.teleport(x, y, theta)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## Video
Para un mayor detalle de lo realizado, se puede ver el video de la simulación [aquí](https://www.canva.com/design/DAG52pnitTc/oZFDkSpRAI9OfWAU71Etqg/watch?utm_content=DAG52pnitTc&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h575623601e).
