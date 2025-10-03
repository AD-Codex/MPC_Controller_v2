
# MPCC controller                    
# contour controlling               -------- 1

# convert to robot frame
# apply MPCC controler
# convert to the world frame



# ---------------------- state equations ----------------------------------------
# x_k+1 = x_k + v_k.cos(theta_k).dt
# y_k+1 = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt

# | X_k+1     |   | 1  0  0 | | X_k     |     | cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1  0 |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0  1 | | theta_k |     |               0  dt | 


# ----------------- linearize state equations ------------------------------------
# x_k+1 = x_k  +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
# y_k+1 = y_k  +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
# theta_k+1 = theta_k + w_k.dt

# | X_k+1     |   | 1  0  -v_k.sin(theta_k).dt | | X_k     |     | cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1   v_k.cos(theta_k).dt |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0                     1 | | theta_k |     |               0  dt |

#     X_1    =           A_0                 . X_0         +          B_0            . U_0
#     X_2    =           A_1                 . X_1         +          B_1            . U_1


# ----------------------- STATE MATRIX ------------------------------------------
# X_predict = phi . x_0 + tau .U_predict  

# X_predict = [ x_1, y_1, theta_1, x_2, y_2, theta_2, x_3, y_3, theta_3, x_4, y_4, theta_4] ^ T

# phi = [          A0]
#       [       A1.A0]
#       [    A2.A1.A0]
#       [ A3.A2.A1.A0]

# tau = [          B0,         0,       0,      0]
#       [       A1.B0,        B1,       0,      0]    
#       [    A2.A1.B0,     A2.B1,      B2,      0]  
#       [ A3.A2.A1.B0,  A3.A2.B1,   A3.B2,     B3]


# ------------------    contour control --------------------------------
# error_2 - perpendiculer distance
# error_1 - parallel distance
# error_0 - orientation error

# error_2 = [ -sin(theta_r)   cos(theta_r)   0].[         x_1 - x_ref ]
#                                               [         y_1 - y_ref ]
#                                               [ theta_1 - theta_ref ]
# error_1 = [  cos(theta_r)   sin(theta_r)   0].[         x_1 - x_ref ]
#                                               [         y_1 - y_ref ]
#                                               [ theta_1 - theta_ref ]
# error_0 = [  0   0   1].[         x_1 - x_ref ]
#                         [         y_1 - y_ref ]
#                         [ theta_1 - theta_ref ]

# | error_2 |    | -sin(theta_r)   cos(theta_r)   0 |   |         x_1 - x_ref |
# | error_1 | =  |  cos(theta_r)   sin(theta_r)   0 | . |         y_1 - y_ref |
# | error_0 |    |             0              0   1 |   | theta_1 - theta_ref |
 
#  [ Error ]  =                   S                   .    [ X_pred - X_ref]

#  [ Error ]^T . Q . [ Error ] = [ X_pred - X_ref]^T  .  S^T  .  Q  .  S  .  [ X_pred - X_ref]

# J = [ Error ]^T . Q . [ Error ] + U_predict^T . R . U_predict
# J = [ X_pred - X_ref]^T  .  Q_  .  [ X_pred - X_ref] + U_predict^T . R . U_predict
# Q_ = S^T  .  Q  .  S 




import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
import QP_matrix as QPFn
import Frame_convert as Fc
import math


class MPC_controller(Node):
    
    def __init__(self):
        super().__init__('MPC_viewer')

        # Initialize veriable
        self.init_state()

        # Publishers
        self.refMarker_pub      = self.create_publisher(MarkerArray, '/object/refernce_point', 10)
        self.predictMarker_pub  = self.create_publisher(MarkerArray, '/object/predict_point', 10)
        
        # Timer
        timer_period = 0.1
        self.timer = self.create_timer( timer_period, self.timer_callback)

        
    def init_state(self):
        self.get_logger().info("Initializing state variables")

        self.X_0 = np.array([ [0.0], [-0.1], [0.0]])
        self.dt = 0.05

        # reference state values [[ x],[ y],[ z]]
        self.ref_state_val = np.array([ [0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]])

        # U_predict [ [v], [w]]
        self.pred_control_val = np.array([  [ 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
                                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


        # control_val_R = np.zeros( len(pred_control_val[0])*2)
        self.control_val_R = np.identity( len( self.pred_control_val[0])*2) *0.05

        self.state_val_Q = np.array([1,1,0.05, 5,5,0.05, 5,5,0.1, 10,10,0.1, 10,10,0.15, 15,15,0.15, 15,15,0.2, 20,20,0.2, 20,20,0.25, 25,25,0.25, 25,25,0.3, 30,30,0.3, 30,30,0.35, 35,35,0.35, 40,40,0.4])
        self.state_val_Q = np.diag( self.state_val_Q)


    def mpc_solver(self, start_state, ref_state):
        self.get_logger().info("Solving the MPC controller")

        while (True):
            control_val, state_value= QPFn.QP_solutions( start_state, self.dt, self.pred_control_val, ref_state, self.control_val_R, self.state_val_Q)
            if ( np.isnan( control_val[0][0])) :
                print( control_val[0][0], type( control_val[0][0]))
            else :
                break

        return control_val, state_value


    def timer_callback(self):
        # Publish reference and predicted marker arrays

        # convert initila robot coord and refernce coords to robot frame
        o_state, RSV_Robot = Fc.Convert_To_Robot_Frame( self.X_0.copy(), self.ref_state_val.copy())
        
        # mpc solve
        control_val, state_value = self.mpc_solver( o_state.copy(), RSV_Robot.copy())

        # convert refernce coords and predicted coords to World frame
        _, RSV_RobotToWorld = Fc.Convert_To_World_Frame( self.X_0.copy() , RSV_Robot.copy())
        _, SV_World = Fc.Convert_To_World_Frame( self.X_0.copy(), state_value.copy())
 
        self.refMarker_pub.publish( self.reference_markers( RSV_RobotToWorld))
        self.predictMarker_pub.publish( self.predicted_markers( SV_World))


    def reference_markers(self, ref_val):
        marker_array = MarkerArray()
        for i in range(len(ref_val[0])):
            marker = self.make_marker(i, ref_val[0][i], ref_val[1][i], ref_val[2][i], [1.0, 0.0, 0.0], 0.4)
            marker_array.markers.append(marker)
        return marker_array
    

    def predicted_markers(self, pre_val):
        marker_array = MarkerArray()
        for i in range(len(pre_val[0])):
            marker = self.make_marker(1000 + i, pre_val[0][i], pre_val[1][i], pre_val[2][i], [0.0, 1.0, 0.0], 0.5)
            marker_array.markers.append(marker)
        return marker_array
    

    def make_marker(self, marker_id, x, y, theta, color_rgb, z_height):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_path"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color.r, marker.color.g, marker.color.b = color_rgb
        marker.color.a = 1.0

        marker.pose.position.x = x * 10
        marker.pose.position.y = y * 10
        marker.pose.position.z = z_height

        r1 = R.from_euler('xyz', [0, 0, -theta])
        r2 = R.from_euler('xyz', [math.pi, 0, 0])
        r_combined = r2 * r1
        q = r_combined.as_quat()  # [x, y, z, w]

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        return marker
    



def main( args=None):
    rclpy.init(args=args)

    MPC_viewer = MPC_controller()

    rclpy.spin(MPC_viewer)

    MPC_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()