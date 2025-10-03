
# MPC Controller
# a Static object avoiding, short range

# convert to robot frame
# singal object avoiding
# apply MPC

 

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
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import QP_constraintsMatrix as QPFn_c
import Frame_convert as Fc
import math
import time


class MPC_controller(Node):
    
    def __init__(self):
        super().__init__('MPC_viewer')

        # Publishers
        self.refMarker_pub      = self.create_publisher(MarkerArray, '/object/refernce_point', 10)
        self.predictMarker_pub  = self.create_publisher(MarkerArray, '/object/predict_point', 10)
        self.staticObj_pub      = self.create_publisher(Marker, '/object/ststic_obj', 10)
        

        # Timer
        timer_period = 0.1
        self.timer = self.create_timer( timer_period, self.timer_callback)

        # Initialize veriable
        self.init_state()

    
    def init_state(self):
        self.get_logger().info("Initializing state variables")

        self.states = 5
        self.X_0    = np.array([[0], [0.1], [0.0]])
        self.staticObj = np.array([[0.5],[0.1]])
        self.statObjDistance = ( (self.staticObj[0][0] - self.X_0[0][0])**2 + (self.staticObj[1][0] - self.X_0[1][0])**2 )**(1/2)

        self.initCoord = np.vstack( (self.X_0, np.array([[self.statObjDistance],[self.staticObj[1][0] - self.X_0[1][0]]])))

        self.dt     = 0.025

        # U_predict [ [v], [w]]
        self.pred_control_val = np.tile( [[1],[0]], 15)

        self.control_val_R = np.zeros( len( self.pred_control_val[0])*2)
        # self.control_val_R = np.identity( len( self.pred_control_val[0])*2) *0.05

        # reference state values [[ x],[ y],[ z]]
        self.ref_state_val = np.array([ [0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]])


        self.state_val_Q = np.array([   [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8],
                                        [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8],
                                        [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40],
                                        [    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                        [    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0] ])


        self.x_distance = self.staticObj[0][0] - self.X_0[0][0]
        # steps = int( radius / 0.05)
        self.steps = 2
        # if ly_ref positive (+0.1) path goes under, else ly_ref negative (-0.1) path goes above
        self.space_val = -0.1

        for i in range( len( self.ref_state_val[0])):
            if ( self.x_distance - self.ref_state_val[0][i] < 0.01 ):
                print(i)
                for j in range( self.steps+1):
                    self.ref_state_val[3][i - j] = self.space_val
                    self.ref_state_val[3][i + j] = self.space_val
                    self.ref_state_val[4][i - j] = self.space_val + j*0.02
                    self.ref_state_val[4][i + j] = self.space_val + j*0.02

                    self.state_val_Q[0][i-j] = 100
                    self.state_val_Q[0][i-j] = 100

                    self.state_val_Q[3][i+j] = 100
                    self.state_val_Q[3][i-j] = 100
                    self.state_val_Q[4][i+j] = 100
                    self.state_val_Q[4][i-j] = 100
                break


        print( self.ref_state_val)
        print( self.state_val_Q)

        self.state_val_Q = self.state_val_Q.flatten(order='F')
        self.state_val_Q = np.diag(self.state_val_Q)

        time.sleep(0.01)


    def mpc_solver( self, states, start_state, ref_state):
        self.get_logger().info("Solving the MPC controller")

        while (True):
            control_val, state_value= QPFn_c.QP_solutions( states, start_state, self.pred_control_val, ref_state, self.control_val_R, self.state_val_Q, self.dt)
            if ( np.isnan(control_val[0][0])) :
                print(control_val[0][0], type(control_val[0][0]))
            else :
                break

        return control_val, state_value
    

    def timer_callback(self):
        # Publish reference and predicted marker arrays

        # convert initila robot coord and refernce coords to robot frame
        o_state_Robot, RSV_Robot, staticObj_Robot = Fc.Convert_To_Robot_Frame2( self.X_0.copy(), self.ref_state_val.copy(), self.staticObj.copy() )
        
        statObj_R_Distance = ( (staticObj_Robot[0][0])**2 + (staticObj_Robot[1][0])**2 )**(1/2)
        initCoord_Robot = np.vstack( (o_state_Robot, np.array([ [statObj_R_Distance],[staticObj_Robot[1][0]] ]) ) )

        # mpc solve
        control_val, state_value = self.mpc_solver( self.states, initCoord_Robot.copy(), RSV_Robot.copy())

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
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_obj"

        # Shape (mesh resource type - 10)
        marker.type = Marker.CYLINDER
        marker.id = 4000
        marker.action = Marker.ADD

        # Scale
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1.0

        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Pose
        marker.pose.position.x = float( obj_val[0] *10.0)
        marker.pose.position.y = float( obj_val[1] *10.0)
        marker.pose.position.z = 0.5

        return marker


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