#!/usr/bin/env python3
"""
telemetry_monitor.py - Nodo 3 del proyecto UAV Control - Challenge.

Se suscribe a los tópicos de telemetría que publica drone_connector y muestra
los datos en consola de forma legible y actualizada.
    - /battery_status (std_msgs/Int32)   nivel de batería (%)
    - /telemetry      (std_msgs/String)  JSON con altura, velocidades, IMU, etc.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')

        # Cache del último valor recibido. Imprimimos cuando llega telemetría.
        self.last_battery = None

        self.create_subscription(Int32,  '/battery_status', self.battery_cb,   10)
        self.create_subscription(String, '/telemetry',      self.telemetry_cb, 10)

        self.get_logger().info('Telemetry monitor iniciado. Esperando datos...')

    def battery_cb(self, msg: Int32):
        self.last_battery = msg.data

    def telemetry_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Telemetría no es JSON válido: {msg.data}')
            return

        battery_str = f'{self.last_battery}%' if self.last_battery is not None else 'N/A'

        # Formateo legible en consola. El \033[2J\033[H limpia pantalla cada update.
        print('\033[2J\033[H', end='')
        print('=' * 50)
        print('         TELEMETRÍA DEL DRON TELLO')
        print('=' * 50)
        print(f"  Batería:           {battery_str}")
        print(f"  Altura (ToF):      {data.get('height_cm', '-')} cm")
        print(f"  Distancia ToF:     {data.get('tof_mm',    '-')} mm")
        print(f"  Altitud baro:      {data.get('baro_m',    '-')} m")
        print('-' * 50)
        print(f"  Velocidad X:       {data.get('speed_x', '-')}")
        print(f"  Velocidad Y:       {data.get('speed_y', '-')}")
        print(f"  Velocidad Z:       {data.get('speed_z', '-')}")
        print('-' * 50)
        print(f"  Pitch / Roll / Yaw: "
              f"{data.get('pitch','-')}° / {data.get('roll','-')}° / {data.get('yaw','-')}°")
        print('-' * 50)
        print(f"  Tiempo de vuelo:   {data.get('flight_time_s', '-')} s")
        print(f"  Temp min / max:    "
              f"{data.get('temp_low_c','-')}°C / {data.get('temp_high_c','-')}°C")
        print('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
