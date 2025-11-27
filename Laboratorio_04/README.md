# Laboratorio_04
* Sergio Avellaneda Piñeros
* David Santiago Cuellar Lopez
* Brayan Yesid Santos Gonzalez

## Documentación 
*Documente la metodologia (procedimiento realizado), los resultados (desiciones), analisis y conclusiones (funcionamiento general)*

1
<img width="1322" height="636" alt="image" src="https://github.com/user-attachments/assets/b338ace6-9d5a-4234-9a45-81bd99236fc5" />

2

<img width="1251" height="622" alt="image" src="https://github.com/user-attachments/assets/c61cbc42-3b2c-49a3-9f64-85b66307eda4" />

3

<img width="1269" height="630" alt="image" src="https://github.com/user-attachments/assets/c31c1355-2dd6-4b9e-9223-35db81ba6957" />





## Diagrama de flujo

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
