#Actioon server
import threading
import time
from navegacion_interfaces.action import Navegacion
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

#Comuniacion Kobuki
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pose import Pose
from rosanautica import RosaNautica

#Transformar a grados
from tf_transformations import euler_from_quaternion
from math import degrees, sqrt

class MoveActionServer(Node):
    """Minimal action server that processes one goal at a time."""
    def __init__(self):
        super().__init__('minimal_action_server')
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.pose = Pose()
        self._action_server = ActionServer(
            self,
            Navegacion,
            'mover',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        #Creamos el subscriptor del odometro
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.listener_callback, 5)
        self.pub = self.create_publisher(Twist, 'commands/velocity', 10)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Peticion de movimiento aceptada')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Peticion de cancelacion aceptada')
        #Poner la kobuki sin movimiento
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        self.pub.publish(move_cmd)
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Ejecutando movimiento...')


        #Leer los parametros de inicio y objetivo 
        th0 = 0
        x0  = 0
        y0  = 0
        camino = RosaNautica().trayectoria_movimiento(1, 0, goal_handle.request.x, goal_handle.request.y)
        print(camino)

        # Creando mensaje de feedback
        feedback_msg = Navegacion.Feedback()
        feedback_msg.pasos_faltantes = len(camino)
        self.get_logger().info('Publicando feedback: {0}'.format(feedback_msg.pasos_faltantes))


        # Creando mensaje de resultado
        result = Navegacion.Result()
        result.termino = False 

        # Empieza a mandar y recibir datos de movimiento
        for paso in camino:
            #Handle de los movimientos abortados y cancelados
            if not goal_handle.is_active:
                self.get_logger().info('Movimiento abortado')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Movimiento cancelado')
                return result
            
            th_objetivo = 0
            d_objetivo = 0
            #Orientar derecha
            if paso == 'D':
                th_objetivo = 270
                d_objetivo = 0.8
            #Orientar izquierda
            elif paso == 'I':
                th_objetivo = 90
                d_objetivo = 0.8
            #Orientar norte
            elif paso == 'N':
                th_objetivo = 0
                d_objetivo = 0.6
            #Orientar sur
            elif paso == 'S':
                th_objetivo = 180
                d_objetivo = 0.6
            
            #Girar cantidad de grados 
            move_cmd = Twist()
            thi = self.pose.th
            if th_objetivo - self.pose.th > 0: 
                move_cmd.angular.z = 0.3
            else: 
                move_cmd.angular.z = -0.3
            
            
            while abs(th_objetivo - self.pose.th) > 1:
                self.pub.publish(move_cmd)
                time.sleep(0.1)


            for i in range(3): 
                move_cmd = Twist()
                self.pub.publish(move_cmd)
                time.sleep(0.1)
            
            #Avanzar una unidad de distancia
            x_i = self.pose.x
            y_i = self.pose.y
            d = sqrt( pow(self.pose.x-x_i,2) + pow(self.pose.y-y_i,2))


            move_cmd = Twist()
            move_cmd.linear.x = -0.3
            while d < d_objetivo:
                d = sqrt(pow(self.pose.x-x_i,2) + pow(self.pose.y-y_i,2))
                self.pub.publish(move_cmd)
                time.sleep(0.1)

            for i in range(3): 
                move_cmd = Twist()
                self.pub.publish(move_cmd)
                time.sleep(0.1)

            #Cambiar el estado de  
            feedback_msg.pasos_faltantes = feedback_msg.pasos_faltantes -1
            self.get_logger().info('Publicando feedback: {0}'.format(feedback_msg.pasos_faltantes))
            goal_handle.publish_feedback(feedback_msg)
           


        goal_handle.succeed()
        
        move_cmd.linear.x = 0.0
        self.pub .publish(move_cmd)

        result.termino = True
        self.get_logger().info('Regresando resultado: {0}'.format(result.termino))

        return result

    def listener_callback(self, data):
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        e = euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
            data.pose.pose.orientation.z, data.pose.pose.orientation.w))[2]
        
        if degrees(e) > 0: 
            self.pose.th = degrees(e)
        else: 
            self.pose.th = 360+degrees(e)

        print(f"(x:{self.pose.x}, y:{self.pose.y}, th:{self.pose.th})")

def main(args=None):
    rclpy.init(args=args)

    action_server = MoveActionServer()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()