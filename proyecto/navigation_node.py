import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import threading
import os

from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_rotacion, calcular_movimiento_relativo

class NavigationNode(Node):
    def __init__(self):
        super().__init__('student_navigation')
        
        # Suscriptores
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)
        
        # Publicador
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables de estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None
        
        # Memorias de estado para los movimientos relativos
        self.target_theta_relativo = None
        self.pose_inicial_relativa = None 
        
        # Variable para almacenar el texto crudo de la escena
        self.texto_escena = ""
        
        # Variables para comunicar el menú interactivo con el control loop
        self.comando_activo = None
        self.parametros_comando = []
        
        # Timer (El loop de control corre 10 veces por segundo)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de Navegación Estudiantil Iniciado.")

        # Iniciamos el menú en un hilo separado para NO bloquear a ROS2
        self.hilo_menu = threading.Thread(target=self.menu_interactivo, daemon=True)
        self.hilo_menu.start()

    # =======================================================
    # CALLBACKS DE ROS2
    # =======================================================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

    def lidar_callback(self, msg):
        self.last_scan = msg

    # =======================================================
    # WRAPPERS PARA LOS ESTUDIANTES
    # =======================================================
    def leer_distancia_en_angulo(self, grados):
        """Retorna la distancia (en metros) de un ángulo específico del Lidar."""
        return obtener_distancia_angulo(self.last_scan, math.radians(grados))

    def leer_distancia_direccion(self, direccion):
        """
        Retorna la distancia en una dirección cardinal específica:
        'frente', 'atras', 'izquierda', 'derecha'.
        """
        mapa_direcciones = {
            'frente': 0.0,
            'izquierda': 90.0,
            'derecha': 270.0,
            'atras': 180.0
        }
        
        direccion = direccion.lower()
        if direccion in mapa_direcciones:
            return self.leer_distancia_en_angulo(mapa_direcciones[direccion])
        else:
            self.get_logger().error(f"Dirección '{direccion}' no válida.")
            return float('inf')

    def leer_distancias_en_rango(self, grados_min, grados_max):
        """Retorna una lista con todas las detecciones en un rango visual."""
        return obtener_distancias_rango(self.last_scan, grados_min, grados_max)

    def rotar_relativo(self, grados_relativos, tolerancia=0.05):
        """
        Gira el robot de forma relativa (ej: 90 grados a la izquierda).
        Retorna True si la maniobra ya finalizó.
        """
        if self.target_theta_relativo is None:
            # Capturamos el ángulo base al arrancar la maniobra
            self.target_theta_relativo = self.current_theta + math.radians(grados_relativos)
            
        cmd, completado = calcular_rotacion(self.current_theta, self.target_theta_relativo, tolerancia=tolerancia)
        self.cmd_pub.publish(cmd)
        
        if completado:
            # Reseteamos la meta para que el robot pueda volver a girar en el futuro
            self.target_theta_relativo = None 
            
        return completado

    def mover_relativo(self, distancia_x_metros, distancia_y_metros, cono_vision=30, dist_segura=0.3, vel_lineal=0.4):
        """
        Desplazamiento usando Cinemática de Tiempo (Dead Reckoning).
        Evalúa el obstáculo según la dirección (Adelante, Atrás, Lados).
        """
        # 1. Inicializamos el cronómetro al arrancar la orden
        if self.pose_inicial_relativa is None:
            self.pose_inicial_relativa = True # Lo usamos como bandera de inicio
            self.tiempo_maniobra = 0.0        # Cronómetro en segundos

        # 2. Determinar hacia dónde nos vamos a mover para vigilar ESA dirección
        if abs(distancia_x_metros) >= abs(distancia_y_metros):
            if distancia_x_metros >= 0:
                # Movimiento hacia el FRENTE
                cono_despejado = self.leer_distancias_en_rango(-cono_vision, cono_vision)
            else:
                # Movimiento hacia ATRÁS (El Lidar ROS2 va de -180 a 180, unimos los dos extremos)
                cono_despejado = self.leer_distancias_en_rango(180-cono_vision, 180) + self.leer_distancias_en_rango(-180, -180+cono_vision)
        else:
            if distancia_y_metros > 0:
                # Movimiento hacia la IZQUIERDA
                cono_despejado = self.leer_distancias_en_rango(90-cono_vision, 90+cono_vision)
            else:
                # Movimiento hacia la DERECHA
                cono_despejado = self.leer_distancias_en_rango(270-cono_vision, 270+cono_vision)

        # 3. Calcular movimiento basado en tiempo
        cmd, estado = calcular_movimiento_relativo(
            self.tiempo_maniobra,
            distancia_x_metros, distancia_y_metros,
            cono_despejado,
            dist_segura=dist_segura,
            vel_lineal=vel_lineal
        )
        self.cmd_pub.publish(cmd)
        
        # 4. Sumamos el tiempo de este ciclo (nuestro timer general corre a 0.1s)
        self.tiempo_maniobra += 0.1
        
        # 5. Si terminamos o nos bloqueamos, limpiamos todo para el próximo comando
        if estado in ['COMPLETADO', 'BLOQUEADO']:
            self.pose_inicial_relativa = None
            self.tiempo_maniobra = 0.0
            
        return estado

    def cargar_escena(self, numero_escena):
        """
        Lee el archivo de la escena indicada y guarda el texto en self.texto_escena.
        """
        # Calculamos la ruta subiendo un nivel de directorio desde este archivo hasta la carpeta 'data'
        directorio_actual = os.path.dirname(os.path.abspath(__file__))
        ruta_archivo = os.path.join(directorio_actual, '..', 'data', f'Escena-Problema{numero_escena}.txt')
        
        try:
            with open(ruta_archivo, 'r', encoding='utf-8') as archivo:
                self.texto_escena = archivo.read()
            self.get_logger().info(f"Escena {numero_escena} cargada correctamente.")
            # Opcional: imprimir un pedacito para confirmar
            print(f"\n--- Contenido Escena {numero_escena} ---\n{self.texto_escena}\n---------------------------")
        except FileNotFoundError:
            self.get_logger().error(f"No se encontró el archivo: {ruta_archivo}")
        except Exception as e:
            self.get_logger().error(f"Error al leer la escena: {e}")

    # =======================================================
    # BUCLE PRINCIPAL (Área de trabajo del estudiante)
    # =======================================================
    def menu_interactivo(self):
        """Pide input por consola sin interrumpir la recepción de datos de los sensores."""
        while rclpy.ok():
            if self.comando_activo is None:
                print("\n" + "="*35)
                print("--- MENÚ DE NAVEGACIÓN ---")
                print("1. Leer distancia en un ángulo")
                print("2. Leer distancias en un rango")
                print("3. Rotar grados relativos")
                print("4. Mover relativo a la posición (X, Y)")
                print("5. Cargar Escena de texto")
                print("6. Leer distancia por dirección (Frente, Atras, Izquierda, Derecha)")
                print("="*35)
                
                try:
                    opcion = input("Elige una opción (1-6): ")
                    
                    if opcion == '1':
                        angulo = float(input("Ingresa el ángulo (en grados): "))
                        self.parametros_comando = [angulo]
                        self.comando_activo = 1
                    
                    elif opcion == '2':
                        ang_min = float(input("Ángulo mínimo (ej. -30): "))
                        ang_max = float(input("Ángulo máximo (ej. 30): "))
                        self.parametros_comando = [ang_min, ang_max]
                        self.comando_activo = 2
                    
                    elif opcion == '3':
                        grados = float(input("¿Cuántos grados quieres rotar? (Positivo=Izq, Negativo=Der): "))
                        self.parametros_comando = [grados]
                        self.comando_activo = 3
                    
                    elif opcion == '4':
                        x = float(input("Cuánto avanzar en X (Frente/Atrás) [en metros]: "))
                        y = float(input("Cuánto avanzar en Y (Izquierda/Derecha) [en metros]: "))
                        self.parametros_comando = [x, y]
                        self.comando_activo = 4

                    elif opcion == '5':
                        numero = int(input("Ingresa el número de la escena (1-6): "))
                        self.parametros_comando = [numero]
                        self.comando_activo = 5

                    elif opcion == '6':
                        dir_input = input("¿Qué dirección? (frente, atras, izquierda, derecha): ").strip().lower()
                        if dir_input in ['frente', 'atras', 'izquierda', 'derecha']:
                            self.parametros_comando = [dir_input]
                            self.comando_activo = 6
                        else:
                            print("Dirección no válida. Intenta de nuevo.")
                        
                    else:
                        print("Opción no válida. Intenta de nuevo.")
                        
                except ValueError:
                    print("Por favor, ingresa únicamente números válidos.")

    # =======================================================
    # BUCLE PRINCIPAL DE CONTROL
    # =======================================================
    def control_loop(self):
        # Evitar fallos si no hay datos del sensor todavía
        if self.last_scan is None:
            return

        if self.comando_activo == 1:
            dist = self.leer_distancia_en_angulo(self.parametros_comando[0])
            self.get_logger().info(f"Distancia a {self.parametros_comando[0]}°: {dist:.2f} m")
            self.comando_activo = None
            
        elif self.comando_activo == 2:
            distancias = self.leer_distancias_en_rango(self.parametros_comando[0], self.parametros_comando[1])
            self.get_logger().info(f"Distancias detectadas: {distancias}")
            self.comando_activo = None
            
        elif self.comando_activo == 3:
            if self.rotar_relativo(self.parametros_comando[0]):
                self.get_logger().info("Rotación completada exitosamente.")
                self.comando_activo = None
                
        elif self.comando_activo == 4:
            estado = self.mover_relativo(self.parametros_comando[0], self.parametros_comando[1])
            
            if estado == 'COMPLETADO':
                self.get_logger().info("Desplazamiento relativo completado.")
                self.comando_activo = None
            elif estado == 'BLOQUEADO':
                self.get_logger().warn("¡Obstáculo detectado! Ruta bloqueada. Abortando movimiento.")
                self.comando_activo = None
        
        elif self.comando_activo == 5:
            self.cargar_escena(self.parametros_comando[0])
            self.comando_activo = None

        elif self.comando_activo == 6:
            direccion = self.parametros_comando[0]
            dist = self.leer_distancia_direccion(direccion)
            self.get_logger().info(f"Distancia hacia el {direccion.upper()}: {dist:.2f} m")
            self.comando_activo = None

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Detener motores forzosamente si se presiona Ctrl+C
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()