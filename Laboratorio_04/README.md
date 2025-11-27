# Laboratorio_04
* Sergio Avellaneda Piñeros
* David Santiago Cuellar Lopez
* Brayan Yesid Santos Gonzalez

## Documentación 
*Documente la metodologia (procedimiento realizado), los resultados (desiciones), analisis y conclusiones (funcionamiento general)*

## Diagrama de flujo

## Codigo
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
Permite dibujar letras específicas (S, A, P, B, Y, G, D, C, L) usando
únicamente velocidad lineal y angular. Después de cada letra, la tortuga
vuelve a su pose inicial usando el servicio de teleport.
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

        # Cliente para el servicio de teleport absoluto
        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute'
        )

        # Hilo separado para leer el teclado sin bloquear el spin de ROS
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def pose_callback(self, msg: Pose):
        """Callback que guarda la pose actual de la tortuga."""
        self.current_pose = msg

    def get_key(self) -> str:
        """
        Lee una tecla del teclado en modo raw (sin necesidad de Enter).
        Bloquea hasta que se presiona una tecla.
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
        Hilo que escucha continuamente el teclado y dispara
        la función de dibujo correspondiente según la tecla.
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
            elif key == '\x03':  # Ctrl+C para salir
                break

    def teleport(self, x: float, y: float, theta: float):
        """
        Llama al servicio de teleport para ubicar la tortuga en (x, y, theta).
        Espera máximo 2 segundos por la respuesta.
        """
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

    def move_distance(self, distance: float, speed: float = 2.0):
        """
        Mueve la tortuga una distancia dada (en unidades de turtlesim)
        hacia adelante (distance > 0) o hacia atrás (distance < 0),
        usando velocidad lineal constante.
        """
        duration = abs(distance) / speed
        msg = Twist()
        msg.linear.x = speed if distance >= 0 else -speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        # Parar la tortuga
        self.publisher_.publish(Twist())
        time.sleep(0.08)

    def turn_angle(self, angle: float, angular_speed: float = 1.0):
        """
        Gira la tortuga un ángulo dado (en radianes).
        angle > 0: giro antihorario, angle < 0: giro horario.
        """
        duration = abs(angle) / angular_speed
        msg = Twist()
        msg.angular.z = angular_speed if angle >= 0 else -angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        # Parar el giro
        self.publisher_.publish(Twist())
        time.sleep(0.08)

    def save_initial_pose(self):
        """
        Guarda la pose actual de la tortuga para restaurarla después de
        dibujar la letra. Devuelve (x, y, theta) o None si no hay pose.
        """
        if self.current_pose is None:
            self.get_logger().error("Pose no disponible")
            return None
        return (self.current_pose.x, self.current_pose.y, self.current_pose.theta)

    # --- Letras ---  

    def draw_S(self):
        """Dibuja la letra S a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        # Radio efectivo de los semicírculos que forman la S
        radius = 1.5 / 2
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        # Orientar para dirigirse al cuadrante II
        self.turn_angle(math.radians(180))

        # Línea para empezar el trazo superior
        self.move_distance(1.5)

        # Orientar para comenzar la parte superior de la S
        self.turn_angle(math.radians(180))

        # Semicírculo superior (giro a la izquierda)
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        start_time = time.time()
        duration = math.pi / angular_speed  # 180°
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        time.sleep(0.05)

        # Semicírculo inferior (giro a la derecha)
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())

        # Volver a la pose original
        self.teleport(x, y, theta)

    def draw_A(self):
        """Dibuja la letra A a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0  # Altura aproximada de la letra
        beta = math.sqrt(18)  # (no se usa, pero se deja como referencia geométrica)

        # Ir a la base izquierda de la A
        self.turn_angle(math.radians(-120))
        self.move_distance(math.sqrt(13))
        self.teleport(x, y, theta)

        # Trazo vertical hacia abajo
        self.turn_angle(math.radians(-90))
        self.move_distance(h)

        # Dibujar la barra "central" de la A
        self.turn_angle(math.radians(-120))
        self.move_distance(1.5)

        self.teleport(x, y, theta)

    def draw_P(self):
        """Dibuja la letra P a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0  # Altura de la letra
        radius = h / 4  # Radio del semicírculo superior
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        # Línea vertical de la P
        self.turn_angle(math.radians(-90))
        self.move_distance(h)
        self.teleport(x, y, theta)

        # Semicírculo superior de la P
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
        """Dibuja la letra B a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0
        radius = h / 4  # Radio de cada semicírculo
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        # Línea vertical de la B
        self.turn_angle(math.radians(90))
        self.move_distance(h)

        # Semicírculo superior
        self.turn_angle(math.radians(-90))
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        start_time = time.time()
        duration = math.pi / angular_speed
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        time.sleep(0.05)

        # Semicírculo inferior (desplazado hacia abajo)
        self.turn_angle(math.radians(180))
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())

        self.teleport(x, y, theta)

    def draw_Y(self):
        """Dibuja la letra Y a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0  # Largo total aproximado de las ramas

        # Rama izquierda de la Y
        self.turn_angle(math.radians(-90))
        self.move_distance(h)

        # Regresar parcialmente y trazar la rama derecha
        self.turn_angle(math.radians(180))
        self.move_distance(h / 2)
        self.turn_angle(math.radians(40))
        self.move_distance(h / 2)

        self.teleport(x, y, theta)

    def draw_G(self):
        """Dibuja la letra G a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        radius = 1.5
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        # Desplazar un poco la letra hacia adelante
        self.move_distance(1.5)
        self.turn_angle(math.radians(180))

        # Arco principal de la G (270°)
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        duration = (3 * math.pi / 2) / angular_speed  # 270°
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        time.sleep(0.05)

        # Trazo horizontal hacia dentro (la "pata" de la G)
        self.turn_angle(math.radians(90))
        self.move_distance(radius)

        self.teleport(x, y, theta)

    def draw_D(self):
        """Dibuja la letra D a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0
        radius = h / 2
        angular_speed = 1.0
        linear_speed = radius * angular_speed

        # Línea vertical
        self.turn_angle(math.radians(90))
        self.move_distance(h)

        # Semicírculo que forma la parte curva de la D
        self.turn_angle(math.radians(-90))
        duration = math.pi / angular_speed
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())

        self.teleport(x, y, theta)

    def draw_C(self):
        """Dibuja la letra C a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        radius = 1.5
        angular_speed = 1.0  # rad/s
        linear_speed = angular_speed * radius

        # Orientar para comenzar el semicírculo de la C
        self.turn_angle(math.radians(180))

        # Semicírculo de 180° que forma la C
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed  # Giro hacia la izquierda
        duration = math.pi / angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())

        self.teleport(x, y, theta)

    def draw_L(self):
        """Dibuja la letra L a partir de la pose actual."""
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial

        h = 3.0  # Altura de la L

        # Trazo vertical hacia arriba
        self.turn_angle(math.radians(-90))
        self.move_distance(h)

        # Base horizontal hacia la derecha
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


## Video
Para un mayor detalle de lo realizado, se puede ver el video de la simulación [aquí](https://www.canva.com/design/DAG52pnitTc/oZFDkSpRAI9OfWAU71Etqg/watch?utm_content=DAG52pnitTc&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h575623601e).
