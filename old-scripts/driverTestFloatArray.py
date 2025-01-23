import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SimpleJointPublisher(Node):

    def __init__(self):
        super().__init__('simple_joint_publisher')
        
        # Parámetros de la frecuencia de la señal y el intervalo de publicación
        self.declare_parameter('dt', 5)
        self.declare_parameter('sinusoid_frequency', 0.1)
        
        self.dt = self.get_parameter('dt').get_parameter_value().integer_value
        self.sinusoid_frequency = self.get_parameter('sinusoid_frequency').get_parameter_value().double_value

        # Tiempo total para generar la señal
        self.total_time = 0.0

        # Configuración del publicador
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_trajectory_controller/commands',
            10
        )

        # Configuración del temporizador
        self.timer = self.create_timer(
            self.dt / 1000.0,  # Conversión de milisegundos a segundos
            self.timer_callback
        )

    def timer_callback(self):
        # Genera el mensaje con valores para las articulaciones
        msg = Float64MultiArray()
        
        # Ejemplo: trayectoria sinusoidal en la primera articulación
        joint1_value = 0.25 * math.pi * math.sin(2.0 * math.pi * self.sinusoid_frequency * self.total_time)
        msg.data = [joint1_value, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 articulaciones como ejemplo
        
        # Publica el mensaje
        self.command_publisher.publish(msg)
        self.total_time += self.dt / 1000.0  # Incrementa el tiempo

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
