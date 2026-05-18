#!/usr/bin/env python3
"""
object_detector.py - Nodo 6 del proyecto UAV Control - Challenge.

Detecta y cuenta objetos de color ROJO y NEGRO en el flujo de video del
dron. Se suscribe a /tello_image y publica el conteo en /objects_count.

Mejoras respecto a la versión anterior:
    - Rango de negro mucho más estricto: exige V bajo (oscuro) Y S bajo
      (sin color). Esto excluye sombras de madera, paredes oscuras de
      colores y áreas con bajo brillo pero color.
    - Filtro por compacidad: solo aceptamos contornos que llenen al menos
      el 50% de su bounding box. Esto descarta sombras alargadas e
      irregulares.
    - Área mínima mayor (800 px) para 720p; las áreas pequeñas suelen ser
      ruido o reflejos.
    - Morfología más agresiva (kernel 7x7) antes del contorneo.
    - Todos los umbrales son parámetros ROS2 ajustables en runtime:
        ros2 run tello_control object_detector \
            --ros-args -p min_area:=1500 -p black_v_max:=35
"""

import os
os.environ['QT_QPA_PLATFORM']      = 'xcb'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['XDG_RUNTIME_DIR']       = '/tmp'

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()

        # --------- Parámetros ROS2 (ajustables sin recompilar) ----------
        self.declare_parameter('min_area',           800)
        self.declare_parameter('min_compactness',    0.50)  # area / bbox_area
        self.declare_parameter('morph_kernel',       7)
        # Negro: V bajo (oscuro) Y S bajo (sin color saturado)
        self.declare_parameter('black_v_max',        40)
        self.declare_parameter('black_s_max',        60)
        # Rojo: dos rangos H envolviendo 0/180
        self.declare_parameter('red_h_low_max',      10)
        self.declare_parameter('red_h_high_min',     165)
        self.declare_parameter('red_s_min',          120)
        self.declare_parameter('red_v_min',          70)

        # ------------------------- ROS topics ---------------------------
        self.create_subscription(Image,   '/tello_image', self.image_cb,    10)
        self.create_subscription(Float32, '/drone_fps',   self.drone_fps_cb, 10)
        self.count_pub = self.create_publisher(Int32, '/objects_count', 10)

        # FPS tracking
        self._drone_fps      = 0.0
        self._det_fps_count  = 0
        self._det_fps_last   = time.time()
        self._det_fps        = 0.0

        self.get_logger().info(
            'Object detector iniciado (rojo + negro) con filtros mejorados.')

    # -----------------------------------------------------------------
    def _params(self):
        """Lee los parámetros actuales (permite cambiarlos en runtime)."""
        gp = self.get_parameter
        return {
            'min_area':         gp('min_area').value,
            'min_compactness':  gp('min_compactness').value,
            'morph_kernel':     gp('morph_kernel').value,
            'black_v_max':      gp('black_v_max').value,
            'black_s_max':      gp('black_s_max').value,
            'red_h_low_max':    gp('red_h_low_max').value,
            'red_h_high_min':   gp('red_h_high_min').value,
            'red_s_min':        gp('red_s_min').value,
            'red_v_min':        gp('red_v_min').value,
        }

    # -----------------------------------------------------------------
    # FILTRO DE CONTORNOS Y DIBUJO
    # -----------------------------------------------------------------
    def find_and_draw(self, mask, frame, color_name, box_color, params):
        """Encuentra contornos válidos en la máscara, los dibuja y devuelve
        cuántos pasaron los filtros de área Y compacidad."""
        count = 0
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < params['min_area']:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            if bbox_area == 0:
                continue

            # Compacidad: cuánto del bounding box llena el contorno.
            # Un objeto "sólido" → 0.7-1.0. Una sombra alargada → 0.2-0.4.
            compactness = area / bbox_area
            if compactness < params['min_compactness']:
                continue

            count += 1
            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
            label = f'{color_name} {count}'
            cv2.putText(frame, label, (x, max(y - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
        return count

    # -----------------------------------------------------------------
    def drone_fps_cb(self, msg: Float32):
        self._drone_fps = msg.data

    # -----------------------------------------------------------------
    # CALLBACK PRINCIPAL
    # -----------------------------------------------------------------
    def image_cb(self, msg: Image):
        # Calcular FPS del detector
        self._det_fps_count += 1
        now = time.time()
        elapsed = now - self._det_fps_last
        if elapsed >= 1.0:
            self._det_fps = self._det_fps_count / elapsed
            self._det_fps_count = 0
            self._det_fps_last = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Error de cv_bridge: {e}')
            return

        params = self._params()

        # Suavizado leve para reducir ruido del sensor antes del HSV.
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        start_time = time.perf_counter()
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        k = params['morph_kernel']
        kernel = np.ones((k, k), np.uint8)

        # ---------------- ROJO ----------------
        # Dos rangos porque H envuelve en 0/180.
        red_lower1 = np.array([0,
                               params['red_s_min'],
                               params['red_v_min']])
        red_upper1 = np.array([params['red_h_low_max'], 255, 255])
        red_lower2 = np.array([params['red_h_high_min'],
                               params['red_s_min'],
                               params['red_v_min']])
        red_upper2 = np.array([180, 255, 255])

        red_mask = cv2.bitwise_or(cv2.inRange(hsv, red_lower1, red_upper1),
                                  cv2.inRange(hsv, red_lower2, red_upper2))
        # Open (erode + dilate) elimina puntos pequeños de ruido.
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)
        # Close (dilate + erode) cierra huecos internos del objeto.
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        # ---------------- NEGRO ----------------
        # Negro real: V bajo Y S bajo. Una sombra de madera tiene S alto
        # (la madera es marrón saturado), una pared oscura azul tiene S alto,
        # solo el negro de verdad cumple ambos a la vez.
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180,
                                params['black_s_max'],
                                params['black_v_max']])
        black_mask = cv2.inRange(hsv, black_lower, black_upper)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN,  kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # ---------------- Conteo y dibujo ----------------
        n_red   = self.find_and_draw(red_mask,   frame, 'ROJO',
                                     (0,   0, 255), params)
        n_black = self.find_and_draw(black_mask, frame, 'NEGRO',
                                     (255, 255, 0), params)
        total = n_red + n_black

        end_time = time.perf_counter()
        
         # Calculamos la diferencia y la convertimos a milisegundos
        processing_time_ms = (end_time - start_time) * 1000



        # Cabecera con conteos
        cv2.rectangle(frame, (5, 5), (380, 38), (0, 0, 0), -1)
        cv2.putText(frame,
                    f'Rojos: {n_red}   Negros: {n_black}   Total: {total}',
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)

        # Segunda barra con FPS
        cv2.rectangle(frame, (5, 42), (380, 72), (0, 0, 0), -1)
        cv2.putText(frame,
                    f'FPS Detector: {self._det_fps:.1f}   FPS Dron: {self._drone_fps:.1f}',
                    (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 255, 0), 2)

        # Tercera barra con tiempo de procesamiento
        cv2.rectangle(frame, (5, 76), (380, 106), (0, 0, 0), -1)
        cv2.putText(frame,
                    f'Tiempo proc: {processing_time_ms:.2f} ms',
                    (12, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 255, 255), 2)


        cv2.imshow('Object Detector', frame)
        cv2.waitKey(1)

        out = Int32()
        out.data = total
        self.count_pub.publish(out)

    # -----------------------------------------------------------------
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
