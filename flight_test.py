import time
from djitellopy import Tello

# --- Conexión ---
drone = Tello()
drone.connect()

# --- Función para mostrar todos los datos de telemetría ---
def mostrar_telemetria(drone):
    print("=" * 50)
    print("TELEMETRÍA DEL TELLO")
    print("=" * 50)

    # Altura y altitud
    print(f"Altura (ToF):          {drone.get_height()} cm")
    print(f"Altitud barométrica:   {drone.get_barometer():.2f} m")
    print(f"Distancia ToF:         {drone.get_distance_tof()} mm")

    # Velocidades
    print(f"\nVelocidad X:           {drone.get_speed_x()}")
    print(f"Velocidad Y:           {drone.get_speed_y()}")
    print(f"Velocidad Z:           {drone.get_speed_z()}")

    # Aceleración
    print(f"\nAceleración X:         {drone.get_acceleration_x():.2f}")
    print(f"Aceleración Y:         {drone.get_acceleration_y():.2f}")
    print(f"Aceleración Z:         {drone.get_acceleration_z():.2f}")

    # Orientación
    print(f"\nPitch:                 {drone.get_pitch()}")
    print(f"Roll:                  {drone.get_roll()}")
    print(f"Yaw:                   {drone.get_yaw()}")

    # Sistema
    print(f"\nBatería:               {drone.get_battery()}%")
    print(f"Temperatura mínima:   {drone.get_lowest_temperature()}")
    print(f"Temperatura máxima:   {drone.get_highest_temperature()}")
    print(f"Tiempo de vuelo:      {drone.get_flight_time()} s")

try:
    print("Monitoreando datos cada 2 segundos. Presiona Ctrl+C para salir.\n")
    while True:
        mostrar_telemetria(drone)
        time.sleep(2)

except KeyboardInterrupt:
    print("\n Monitoreo detenido.")
    drone.end()