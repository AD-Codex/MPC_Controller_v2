
# MPC Controller with customizable object adding
# Static object avoiding new QPC library
# long range

# convert to robot frame
# we convert original object coord to robot frame. then ly_k is general

 

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


# ----------------------- consider a static object ---------------------------------
#
#                    static object - ( X_s, Y_s)
#                    ..
#                  . .
#                .  .
#              .   .
#            .    .
#     l_k  .     . l_k+1
#        .      . 
#      .        k+1 step - (X_k+1, Y_k+1, theta_k+1)             
#    . 
# K step - (X_k, Y_k, theta_k)      
#   (v_k, w_k)

# orientation between,     K+1 step & K step  =  theta_k
# angle to static object,       alpha_k       =  aniSin[ ly_k/l_k]
# ori & static object angle,                  =  alphs_k - theta_k
#                                     l_k+1   =  l_k  -  v_k. cos( alpha_k - theta_k) . dt    ; consider small dts
#
#                                     
# x_k+1     = x_k + v_k.cos(theta_k).dt
# y_k+1     = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt
# l_k+1     = l_k  -  v_k.cos( alpha_k - theta_k).dt
# ly_k+1    = ly_k - v_k.sin(theta_k).dt

# | X_k+1     |   | 1  0  0  0  0 | | X_k     |     |            cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1  0  0  0 |.| Y_k     |  +  |            sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0  1  0  0 | | theta_k |     |                          0  dt | 
# | l_k+1     | = | 0  0  0  1  0 | | l_k     |     |-cos( alpha_k - theta_k).dt   0 |
# | ly_k+1    | = | 0  0  0  0  1 | | ly_k    |     |           -sin(theta_k).dt   0 |


# ----------------- linearize state equations ------------------------------------
# x_k+1     = x_k     +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
# y_k+1     = y_k     +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
# theta_k+1 = theta_k +  w_k.dt
# l_k+1     = l_k     -  v_k.cos(alpha_k - theta_k).dt  - v_k.sin(alpha_k - theta_k).dt.theta_k - [v_k.sin(alpha_k-theta_k).dt].l_k/[cos(alpha_k).l_k.l_k] + [v_k.sin(alpha_k-theta_k).dt].ly_k/cos(alpha_k)
# ly_k+1    = ly_k    -  v_k.sin(theta_k).dt  -  v_k.cos(theta_k).dt.theta_k


# | X_k+1     |   | 1  0           -v_k.sin(theta_k).dt                                                            0                                                  0 | | X_k     |     |            cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1            v_k.cos(theta_k).dt                                                            0                                                  0 |.| Y_k     |  +  |            sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0                              1                                                            0                                                  0 | | theta_k |     |                          0  dt | 
# | l_k+1     | = | 0  0  - v_k.sin(alpha_k-theta_k).dt  1-[v_k.sin(alpha_k-theta_k).ly_k.dt]/[cos(alpha_k).l_k.l_k]  [v_k.sin(alpha_k-theta_k).dt]/[cos(alpha_k).ly_k] | | l_k     |     |-cos( alpha_k - theta_k).dt   0 |
# | ly_k+1    | = | 0  0           -v_k.cos(theta_k).dt                                                            0                                                  1 | | ly_k    |     |           -sin(theta_k).dt   0 |

#     X_1    =           A_0                 . X_0         +          B_0            . U_0
#     X_2    =           A_1                 . X_1         +          B_1            . U_1


# ----------------------- STATE MATRIX ------------------------------------------
# X_predict = phi . x_0 + tau .U_predict  

# X_predict = [ x_1, y_1, theta_1, l_1, ly_1, x_2, y_2, theta_2, l_2, ly_2 x_3, y_3, theta_3, l_3, ly_3 x_4, y_4, theta_4 l_4, ly_4] ^ T

# phi = [          A0]
#       [       A1.A0]
#       [    A2.A1.A0]
#       [ A3.A2.A1.A0]

# tau = [          B0,         0,       0,      0]
#       [       A1.B0,        B1,       0,      0]    
#       [    A2.A1.B0,     A2.B1,      B2,      0]  
#       [ A3.A2.A1.B0,  A3.A2.B1,   A3.B2,     B3]


# --------------------------- Cost Fn --------------------------------------
# J = (X_predict - X_ref)^T . Q . (X_predict - X_ref) + U_predict^T . R . U_predict
#   = (1/2) . U_predict^T . H . U_predict + f^T . U_predict + constant

# H = tau^T . Q . tau + R
# f = tau^T . Q . ( phi . x_0 - X_ref)


# -------------------------- daqp general method ------------------------------
# (xstar,fval,exitflag,info) = daqp.solve(H,f,A,bupper,blower,sense)



import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
import QPC as QPC
import Frame_convert as Fc
import math
import time
import sys


np.set_printoptions(threshold=sys.maxsize)


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

        # for simulation calculation,1 step -->  0.025 s , 0.05 m ,   2 m/s
        # for real time calculation, 1 step -->      1 s ,  0.5 m , 0.5 m/s

        # number of states
        self.states = 7

        # starting point
        self.X_0 = np.array([[0], [0.05], [0]])

        # object coordinates
        self.staticObj = np.array([ [  0.5,  0.8],
                                    [    0,    0],
                                    [    0,    0] ])

        # starting point to object distane
        for i in range( len( self.staticObj[0])):
            self.staticObj[2][i] = ( (self.staticObj[0][i] - self.X_0[0][0])**2 + (self.staticObj[1][i] - self.X_0[1][0])**2 )**(1/2)

        # predicted control states (number)
        self.pred_control_val = np.tile( [[1],[0]], 20)
        # cost Fn control state constant
        self.control_val_R = np.zeros( len(self.pred_control_val[0])*2)

        # step time seconds
        self.dt = 0.025

        # reference values, (numer + 1)
        self.ref_state_val = np.array([ [0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75,  0.8, 0.85,  0.9, 0.95,    1], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0] ])


        # cost fn state value constant (number)
        self.state_val_Q = np.array([   [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   10],
                                        [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   10],
                                        [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40, 0.45, 0.45, 0.50, 0.50] ])


        for i in range( self.states - 3):
            self.ref_state_val = np.vstack(( self.ref_state_val, np.zeros(21)))       # number +1
            self.state_val_Q = np.vstack(( self.state_val_Q, np.zeros(20)))           # number


        # update ref_state_val and state_val_Q -----------------------------
        space_val = [ 0.1, 0.1]
        steps = 2
        weight = 100
        for objs in range( len(  self.staticObj[0])):
            for i in range( len( self.ref_state_val[0])):
                if ( ( self.staticObj[0][objs] - self.X_0[0][0]) - self.ref_state_val[0][i] < 0.01 ):
                    for j in range( steps +1):
                        # if ly_ref positive (+0.1) path goes under, else ly_ref negative (-0.1) path goes above
                        self.ref_state_val[ 2*(objs+1) + 1 ][i - j] = space_val[objs]
                        self.ref_state_val[ 2*(objs+1) + 1 ][i + j] = space_val[objs]
                        self.ref_state_val[ 2*(objs+1) + 2 ][i - j] = space_val[objs] + j*0.02
                        self.ref_state_val[ 2*(objs+1) + 2 ][i + j] = space_val[objs] + j*0.02

                        self.state_val_Q[0][i-j] = weight
                        self.state_val_Q[0][i-j] = weight

                        self.state_val_Q[ 2*(objs+1) + 1 ][i+j] = weight
                        self.state_val_Q[ 2*(objs+1) + 1 ][i-j] = weight
                        self.state_val_Q[ 2*(objs+1) + 2 ][i+j] = weight
                        self.state_val_Q[ 2*(objs+1) + 2 ][i-j] = weight
                    break


        # print( self.state_val_Q)
        self.state_val_Q = self.state_val_Q.flatten(order='F')
        self.state_val_Q = np.diag( self.state_val_Q)

        time.sleep(0.01)

    
    def mpc_solver( self, states, start_state, ref_state):
        self.get_logger().info("Solving the MPC controller")

        while (True):
            control_val, state_value= QPC.QP_solutions( states, start_state, self.pred_control_val, ref_state, self.control_val_R, self.state_val_Q, self.dt)
            if ( np.isnan( control_val[0][0])) :
                print( control_val[0][0], type( control_val[0][0]))
            else :
                break

        return control_val, state_value
    

    def timer_callback(self):
        # Publish reference and predicted marker arrays

        # convert initial value to robot frame
        o_state_Robot, RSV_Robot , staticObj_Robot = Fc.Convert_To_Robot_Frame2( self.X_0.copy(), self.ref_state_val.copy(), self.staticObj.copy())
        for i in range( len(staticObj_Robot[0])):
            # initialize coordinate of system
            o_state_Robot = np.vstack( (o_state_Robot, np.array([ [staticObj_Robot[2][i]], [staticObj_Robot[1][i]] ]) ))
        
        # mpc solver
        control_val, state_value = self.mpc_solver( self.states, o_state_Robot, RSV_Robot)

        # inti_state, ref_state_val = Fc.Convert_To_World_Frame( X_0, ref_state_val, staticObj)


        # publish nodes
        self.refMarker_pub.publish( self.reference_markers( RSV_Robot ))
        self.predictMarker_pub.publish( self.predicted_markers( state_value ))
        self.staticObj_pub.publish( self.object_marker( staticObj_Robot ))


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


    def object_marker(self, obj_val):
        print(obj_val)
        marker_array = MarkerArray()
        for i in range( len(obj_val[0])):
            marker = self.make_objmarker( 4000 +i, obj_val[0][i], obj_val[1][i], [1.0, 0.0, 0.0])
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
    

    def make_objmarker(self, marker_id, x, y, color_rgb):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_obj"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1.0

        marker.color.r, marker.color.g, marker.color.b = color_rgb
        marker.color.a = 0.5

        marker.pose.position.x = float( x *10)
        marker.pose.position.y = float( y *10)
        marker.pose.position.z = 0.5

        return marker




def main( args=None):
    rclpy.init(args=args)

    MPC_viewer = MPC_controller()

    rclpy.spin(MPC_viewer)

    MPC_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




#  ros2 launch foxglove_bridge foxglove_bridge_launch.xml