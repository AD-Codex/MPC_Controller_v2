
# New QPC_dyanmic library with contour controle
# Only path following
# robot frame calculations



# ---------------------- state equations ----------------------------------------
#
#                       k+1 step
#                      .
#                   .
#                .
#             .
#          .
#       .
#     K step - (X_k, Y_k, theta_k) 
#  ( v_k, w_k)

# x_k+1 = x_k + v_k.cos(theta_k).dt
# y_k+1 = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt

# | X_k+1     |   | 1  0  0 | | X_k     |     | cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1  0 |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0  1 | | theta_k |     |               0  dt | 


# ----------------------- consider a dynanic object ---------------------------------
# object move toward to robot
#
#                                (Xs_k, Ys_k)
#         (Xs_k+1, Ys_k+1)      .
#                       .    .
#                      .  . 
#                     . .
#                   ..
#                .  .
#     l_k     .    . l_k+1
#          .      . 
#       .           k+1 step - (X_k+1, Y_k+1, theta_k+1)             
#    .     
# K step - (X_k, Y_k, theta_k)      
#
# robot movement    ( v_k, w_k)
# object movement   ( X_dot_k, Y_dot_k), positive if move toward robot

# robot to object distance,     l_(k+1)_x   =   l_k_x - X_dot_k.dt - v_k.cos(theta_k).dt
#                               l_(k+1)_y   =   l_k_y - Y_dot_k.dt - v_k.sin(theta_k).dt

#                                     
# x_k+1     = x_k + v_k.cos(theta_k).dt
# y_k+1     = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt
# l_(k+1)_x = l_k_x - v_k.cos(theta_k).dt - X_dot_k.dt
# l_(k+1)_y = l_k_y - v_k.sin(theta_k).dt - Y_dot_k.dt 

# | X_k+1     |   | 1  0  0  0  0 | | X_k     |     |  cos(theta_k).dt   0 | | V_k |     |   0    0 | | X_dot_k |
# | Y_k+1     | = | 0  1  0  0  0 |.| Y_k     |  +  |  sin(theta_k).dt   0 |.| w_k |  +  |   0    0 |.| Y_dot_k |
# | theta_k+1 |   | 0  0  1  0  0 | | theta_k |     |                0  dt |             |   0    0 |
# | l_(k+1)_x | = | 0  0  0  1  0 | | l_k_x   |     | -cos(theta_k).dt   0 |             | -dt    0 |
# | l_(k+1)_y | = | 0  0  0  0  1 | | l_k_y   |     | -sin(theta_k).dt   0 |             |   0  -dt |
 

# ----------------- linearize state equations ------------------------------------
# x_k+1     = x_k     +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
# y_k+1     = y_k     +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
# theta_k+1 = theta_k +  w_k.dt
# l_(k+1)_x = l_k     -  v_k.cos(theta_k).dt  +  v_k.sin(theta_k).dt.theta_k - X_dot_k.dt
# l_(k+1)_y = ly_k    -  v_k.sin(theta_k).dt  -  v_k.cos(theta_k).dt.theta_k - Y_dot_k.dt 


# | X_k+1     |   | 1  0  -v_k.sin(theta_k).dt  0   0 | | X_k     |     |  cos(theta_k).dt   0 | | V_k |     |   0    0 | | X_dot_k |
# | Y_k+1     | = | 0  1   v_k.cos(theta_k).dt  0   0 |.| Y_k     |  +  |  sin(theta_k).dt   0 |.| w_k |  +  |   0    0 |.| Y_dot_k |
# | theta_k+1 |   | 0  0                     1  0   0 | | theta_k |     |                0  dt |             |   0    0 |
# | l_(k+1)_x | = | 0  0   v_k.sin(theta_k).dt  1   0 | | l_k_x   |     | -cos(theta_k).dt   0 |             | -dt    0 |
# | l_(k+1)_y | = | 0  0  -v_k.cos(theta_k).dt  0   1 | | l_k_y   |     | -sin(theta_k).dt   0 |             |   0  -dt |


#     X_1       =                                 A_0  .  X_0       +                      B_0  .  U_0    +         D_0  .  W_0
#     X_2       =                                 A_1  .  X_1       +                      B_1  .  U_1    +         D_1  .  W_1


# ----------------------- STATE MATRIX ------------------------------------------
# X_predict = phi . x_0 + tau .U_predict +  eta .W_value

# X_predict = [ x_1, y_1, theta_1, l_1, ly_1, x_2, y_2, theta_2, l_2, ly_2 x_3, y_3, theta_3, l_3, ly_3 x_4, y_4, theta_4 l_4, ly_4] ^ T

# phi = [          A0]
#       [       A1.A0]
#       [    A2.A1.A0]
#       [ A3.A2.A1.A0]

# tau = [          B0,         0,       0,      0]
#       [       A1.B0,        B1,       0,      0]    
#       [    A2.A1.B0,     A2.B1,      B2,      0]  
#       [ A3.A2.A1.B0,  A3.A2.B1,   A3.B2,     B3]

# eta = [          D0,         0,       0,      0]
#       [       A1.D0,        D1,       0,      0]    
#       [    A2.A1.D0,     A2.D1,      D2,      0]  
#       [ A3.A2.A1.D0,  A3.A2.D1,   A3.D2,     D3]


# --------------------------- Cost Fn --------------------------------------
# J = (X_predict - X_ref)^T . Q . (X_predict - X_ref) + U_predict^T . R . U_predict
#   = (1/2) . U_predict^T . H . U_predict + f^T . U_predict + constant

# H = tau^T . Q . tau + R
# f = tau^T . Q . ( phi . x_0 + eta . W_value - X_ref)


# -------------------------- daqp general method ------------------------------
# (xstar,fval,exitflag,info) = daqp.solve(H,f,A,bupper,blower,sense)



import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
import QPC_dynamic as QPC
import Frame_convert as Fc
import math
import time
import sys


# np.set_printoptions( threshold=sys.maxsize)
np.set_printoptions(precision=3, suppress=True)



class MPC_controller(Node):

    def __init__(self):
        super().__init__('MPC_viewer')

        # Initialize veriable
        self.init_state()

        # Publishers
        self.refMarker_pub      = self.create_publisher(MarkerArray, '/object/refernce_point', 10)
        self.predictMarker_pub  = self.create_publisher(MarkerArray, '/object/predict_point', 10)
        self.staticObj_pub      = self.create_publisher(MarkerArray, '/object/static_obj', 10)

        # Timer
        timer_period = 0.1
        self.timer = self.create_timer( timer_period, self.timer_callback)

 
    def init_state(self):
        self.get_logger().info("Initilizing state variable")
        

        # robot states initializing ...............................................................
        self.robot_init = np.array([[0], [0.1], [0]] , dtype=np.float64)

        # reference values, (numer + 1)
        self.ref_state_val  = np.array([[0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75,  0.8], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]] , dtype=np.float64)
        
        # cost fn state value constant (number)
        self.state_val_Q    = np.array( [ [  0.5,  0.5,    1,    1,  1.5,  1.5,    2,    2,  2.5,  2.5,    3,    3,  3.5,  3.5,    8,    8],
                                          [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8],
                                          [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40]] , dtype=np.float64)
        # diagonal matrix
        self.diag_state_val_Q = self.state_val_Q.flatten(order='F')
        self.diag_state_val_Q = np.diag( self.diag_state_val_Q)
        
        # predicted control states (number)
        self.pred_control_val   = np.tile( [[1],[0]], 16)

        # cost Fn control state constant
        self.control_val_R      = np.zeros( len(self.pred_control_val[0])*2 , dtype=np.float64)
        # self.control_val_R = np.identity( len( self.pred_control_val[0])*2) *0.05


        # object states initializing ................................................................
        self.staticObj  = np.array([ []] , dtype=np.float64)
        
        self.obj_vel    = np.array([ [ ]] , dtype=np.float64)

        

        # controller state initializing .............................................................
        self.O_state = self.robot_init
        for obj in range( int( len(self.staticObj)/2)):
            obj_state = self.staticObj[ obj*2: (obj+1)*2].flatten()
            self.O_state = np.vstack(( self.O_state, np.array( [ [ obj_state[0]-self.robot_init[0][0]],[ obj_state[1]-self.robot_init[1][0]]])))
        # print( self.O_state)

        self.num_states = 3
        self.pred_horizon = 16

        # step time seconds
        self.dt = 0.025






    def timer_callback(self):

        init_state, ref_state_val = Fc.Convert_To_Robot_Frame( self.robot_init.copy(), self.ref_state_val.copy())

        control_val, state_val = QPC.QPC_solutions( self.num_states,
                                                    self.pred_horizon,
                                                    init_state,
                                                    self.pred_control_val.copy(),
                                                    self.obj_vel.copy(),
                                                    ref_state_val.copy(),
                                                    self.control_val_R.copy(),
                                                    self.diag_state_val_Q.copy(),
                                                    self.dt)
        
        # publish nodes
        self.predictMarker_pub.publish( self.predicted_markers( state_val))
        self.refMarker_pub.publish( self.reference_markers( ref_state_val ))



    def reference_markers(self, ref_val):
        marker_array = MarkerArray()
        for i in range(len(ref_val[0])):
            marker = self.make_marker(i, ref_val[0][i], ref_val[1][i], ref_val[2][i], [1.0, 0.0, 0.0], 0.4)
            marker_array.markers.append(marker)
            # Create the text marker to display step number
            text_marker = self.make_textmarker( (i+500), ref_val[0][i], ref_val[1][i], 0.5, [1.0, 0.0, 0.0], str(i + 1))
            marker_array.markers.append(text_marker)
        return marker_array

    def predicted_markers(self, pre_val):
        marker_array = MarkerArray()
        for i in range(len(pre_val[0])):
            marker = self.make_marker(1000 + i, pre_val[0][i], pre_val[1][i], pre_val[2][i], [0.0, 1.0, 0.0], 0.5)
            marker_array.markers.append(marker)
            # Create the text marker to display step number
            text_marker = self.make_textmarker( (i+1500), pre_val[0][i], pre_val[1][i], 0.5, [0.0, 1.0, 0.0], str(i + 1))
            marker_array.markers.append(text_marker)
        return marker_array

    def object_marker(self, obj_states):
        print(obj_states)
        marker_array = MarkerArray()
        for obj in range( int(len(obj_states)/2)):
            obj_coord = obj_states[ 2*obj:2*obj+2, :]
            for i in range( len(obj_states[0])):
                marker = self.make_objmarker( (2000 + 100*obj + i), obj_coord[0][i], obj_coord[1][i], [0.0, 0.0, 1.0], (1-i/len(obj_states[0])))
                marker_array.markers.append(marker)
                # Create the text marker to display step number
                text_marker = self.make_textmarker( (i+100*obj+5000), obj_coord[0][i], obj_coord[1][i], 0.2, [0.0, 0.0, 1.0], str(i + 1))
                marker_array.markers.append(text_marker)
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
        marker.color.a = 0.5

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
    
    def make_objmarker(self, marker_id, x, y, color_rgb, a):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_obj"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 0.01

        marker.color.r, marker.color.g, marker.color.b = color_rgb
        marker.color.a = a/2

        marker.pose.position.x = float( x *10)
        marker.pose.position.y = float( y *10)
        marker.pose.position.z = 0.1

        return marker
    
    def make_textmarker(self, marker_id, x, y, z, color_rgb, text):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obj_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = float(x * 10)
        marker.pose.position.y = float(y * 10)
        marker.pose.position.z = z

        marker.scale.z = 0.1

        marker.color.r, marker.color.g, marker.color.b = color_rgb
        marker.color.a = 1.0

        marker.text = text

        return marker





def main( args=None):
    rclpy.init(args=args)

    MPC_viewer = MPC_controller()

    rclpy.spin(MPC_viewer)

    MPC_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
