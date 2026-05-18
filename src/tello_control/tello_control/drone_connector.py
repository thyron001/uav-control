#!/usr/bin/env python3
"""
drone_connector.py - Nodo 1 del proyecto UAV Control - Challenge.

Único nodo que se comunica con el Tello vía djitellopy.

Doble canal de comandos:
    - /tello_cmd          → comandos normales (mission_planner). Cola FIFO.
    - /tello_cmd_priority → comandos urgentes (mission_planner failsafe). Saltan la
                            cola y pueden interrumpir movimientos en curso.

Cómo se interrumpen los movimientos:
    Las funciones de djitellopy (move_forward, etc.) bloquean ~3-5 s.
    Cuando llega un comando prioritario, se publica una bandera; el thread
    del comando normal la consulta y, si está activa, NO inicia el siguiente
    movimiento. El dron acepta sin problemas un nuevo comando aunque haya
    uno en curso (responde al último). Para 'land' y 'emergency' enviamos
    DIRECTAMENTE el comando UDP raw al dron sin esperar OK, garantizando
    aterrizaje inmediato.
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from djitellopy import Tello


class DroneConnector(Node):
    def __init__(self):
        super().__init__('drone_connector')

        # ---------------- Estado interno ----------------
        self.bridge = CvBridge()
        self.drone = Tello()
        self.connected = False
        self.flying = False
        self.frame_reader = None

        # Bandera de aborto: cuando mission_planner manda priority (failsafe),
        # esta bandera bloquea cualquier comando normal pendiente.
        self.priority_lock = threading.Lock()
        self.aborted = False

        # ---------------- Callback groups ---------------
        self.timers_group = MutuallyExclusiveCallbackGroup()
        self.cmd_group = ReentrantCallbackGroup()
        self.priority_group = ReentrantCallbackGroup()

        # ---------------- Publicadores ------------------
        self.battery_pub   = self.create_publisher(Int32,   '/battery_status', 10)
        self.telemetry_pub = self.create_publisher(String,  '/telemetry',      10)
        self.status_pub    = self.create_publisher(String,  '/drone_status',   10)
        self.image_pub     = self.create_publisher(Image,   '/tello_image',    10)
        self.fps_pub       = self.create_publisher(Float32, '/drone_fps',      10)

        # FPS tracking
        self._fps_count     = 0
        self._fps_last_time = time.time()

        # ---------------- Suscriptores ------------------
        self.cmd_sub = self.create_subscription(
            String, '/tello_cmd', self.cmd_callback, 10,
            callback_group=self.cmd_group)

        self.priority_sub = self.create_subscription(
            String, '/tello_cmd_priority', self.priority_callback, 10,
            callback_group=self.priority_group)

        # ---------------- Conexión inicial --------------
        self.connect_drone()

        # ---------------- Timers ------------------------
        self.create_timer(1.0,  self.publish_battery,   callback_group=self.timers_group)
        self.create_timer(0.5,  self.publish_telemetry, callback_group=self.timers_group)
        self.create_timer(1.0,  self.publish_status,    callback_group=self.timers_group)
        self.create_timer(1/30, self.publish_image,     callback_group=self.timers_group)

    # ===================================================
    # CONEXIÓN
    # ===================================================
    def connect_drone(self):
        try:
            self.drone.connect()
            self.connected = True
            self.get_logger().info(
                f'Conectado al Tello. Batería inicial: {self.drone.get_battery()}%')
            self.drone.streamon()
            self.frame_reader = self.drone.get_frame_read()
            self.get_logger().info('Stream de video activado.')
        except Exception as e:
            self.connected = False
            self.get_logger().error(f'Error al conectar con el dron: {e}')

    # ===================================================
    # PUBLICADORES PERIÓDICOS
    # ===================================================
    def publish_battery(self):
        if not self.connected:
            return
        try:
            msg = Int32()
            msg.data = int(self.drone.get_battery())
            self.battery_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'No se pudo leer la batería: {e}')

    def publish_telemetry(self):
        if not self.connected:
            return
        try:
            data = {
                'height_cm':       self.drone.get_height(),
                'tof_mm':          self.drone.get_distance_tof(),
                'baro_m':          round(self.drone.get_barometer(), 2),
                'speed_x':         self.drone.get_speed_x(),
                'speed_y':         self.drone.get_speed_y(),
                'speed_z':         self.drone.get_speed_z(),
                'pitch':           self.drone.get_pitch(),
                'roll':            self.drone.get_roll(),
                'yaw':             self.drone.get_yaw(),
                'flight_time_s':   self.drone.get_flight_time(),
                'temp_low_c':      self.drone.get_lowest_temperature(),
                'temp_high_c':     self.drone.get_highest_temperature(),
            }
            msg = String()
            msg.data = json.dumps(data)
            self.telemetry_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'No se pudo leer la telemetría: {e}')

    def publish_status(self):
        msg = String()
        if not self.connected:
            msg.data = 'DISCONNECTED'
        elif self.flying:
            msg.data = 'FLYING'
        else:
            msg.data = 'LANDED'
        self.status_pub.publish(msg)

    def publish_image(self):
        if not self.connected or self.frame_reader is None:
            return
        try:
            frame = self.frame_reader.frame
            if frame is None:
                return
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'tello_camera'
            self.image_pub.publish(img_msg)

            self._fps_count += 1
            now = time.time()
            elapsed = now - self._fps_last_time
            if elapsed >= 1.0:
                fps_msg = Float32()
                fps_msg.data = float(self._fps_count / elapsed)
                self.fps_pub.publish(fps_msg)
                self._fps_count = 0
                self._fps_last_time = now
        except Exception as e:
            self.get_logger().warn(f'Error publicando imagen: {e}')

    # ===================================================
    # COMANDO PRIORITARIO (mission_planner failsafe)
    # ===================================================
    def priority_callback(self, msg: String):
        """Comando urgente del failsafe. Marca la bandera de aborto y ejecuta
        land/emergency DIRECTO al dron, sin pasar por la API bloqueante."""
        if not self.connected:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Priority: JSON inválido: {msg.data}')
            return

        cmd = payload.get('cmd', '').lower()
        self.get_logger().warn(f'>>> PRIORIDAD: {cmd} <<<')

        # Activamos la bandera ANTES de cualquier acción. Esto bloquea
        # cualquier comando normal que esté pendiente o llegando.
        with self.priority_lock:
            self.aborted = True

        try:
            if cmd in ('land', 'emergency'):
                # Enviamos el comando UDP DIRECTO usando send_command_without_return.
                # Esto NO espera respuesta del dron y NO se bloquea esperando que
                # termine un movimiento previo. El dron procesa el comando inmediato.
                if cmd == 'land':
                    self.drone.send_command_without_return('land')
                else:
                    self.drone.send_command_without_return('emergency')
                self.flying = False
                self.get_logger().warn(f'Comando prioritario {cmd} enviado al dron.')
            else:
                self.get_logger().warn(f'Comando prioritario desconocido: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Error en comando prioritario: {e}')

    # ===================================================
    # COMANDO NORMAL (mission_planner)
    # ===================================================
    def cmd_callback(self, msg: String):
        if not self.connected:
            return

        # Si hubo un aborto, ignoramos cualquier comando normal hasta
        # que el sistema vuelva a un estado conocido (LANDED).
        with self.priority_lock:
            if self.aborted:
                if not self.flying:
                    # Ya aterrizamos. Reseteamos para próximas misiones.
                    self.aborted = False
                else:
                    self.get_logger().warn('Comando ignorado: sistema en aborto.')
                    return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'JSON inválido: {msg.data}')
            return

        cmd   = payload.get('cmd', '').lower()
        value = payload.get('value')

        try:
            if cmd == 'takeoff':
                self.drone.takeoff()
                self.flying = True
            elif cmd == 'land':
                self.drone.land()
                self.flying = False
            elif cmd == 'forward':
                self.drone.move_forward(int(value))
            elif cmd == 'back':
                self.drone.move_back(int(value))
            elif cmd == 'left':
                self.drone.move_left(int(value))
            elif cmd == 'right':
                self.drone.move_right(int(value))
            elif cmd == 'up':
                self.drone.move_up(int(value))
            elif cmd == 'down':
                self.drone.move_down(int(value))
            elif cmd == 'rotate_cw':
                self.drone.rotate_clockwise(int(value))
            elif cmd == 'rotate_ccw':
                self.drone.rotate_counter_clockwise(int(value))
            else:
                self.get_logger().warn(f'Comando desconocido: {cmd}')
                return

            self.get_logger().info(
                f'Comando ejecutado: {cmd} {value if value is not None else ""}')
        except Exception as e:
            # No imprimimos como ERROR si fue por aborto del dron.
            self.get_logger().warn(f'Comando "{cmd}" interrumpido o fallido: {e}')

    # ===================================================
    # CIERRE
    # ===================================================
    def destroy_node(self):
        try:
            if self.connected:
                if self.flying:
                    self.drone.send_command_without_return('land')
                self.drone.streamoff()
                self.drone.end()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneConnector()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
