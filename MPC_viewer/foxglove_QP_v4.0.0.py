
# 1. first consider only the following path
# 2. convert to the robot frame
# 3. find if there any collition
# 4. if yes consider the respective states and update the weigth values of object
# 5. apply MPC controller with contour edit



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
        

        # starting point
        self.X_0 = np.array([[0], [0], [0]])

        # object coordinates
        self.staticObj = np.array([ [  0.4,  0.8],
                                    [    0, 0.05],
                                    [    0,    0] ])
        

        # starting point to object distane
        for i in range( len( self.staticObj[0])):
            self.staticObj[2][i] = ( (self.staticObj[0][i] - self.X_0[0][0])**2 + (self.staticObj[1][i] - self.X_0[1][0])**2 )**(1/2)

        # predicted control states (number)
        self.pred_control_val = np.tile( [[1],[0]], 16)

        # cost Fn control state constant
        self.control_val_R = np.zeros( len(self.pred_control_val[0])*2)
        # self.control_val_R = np.identity( len( self.pred_control_val[0])*2) *0.05

        # step time seconds
        self.dt = 0.025

        # reference values, (numer + 1)
        self.ref_state_val = np.array([ [0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75,  0.8], 
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                        [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0] ])


        # cost fn state value constant (number)
        self.state_val_Q = np.array([   [  0.5,  0.5,    1,    1,  1.5,  1.5,    2,    2,  2.5,  2.5,    3,    3,  3.5,  3.5,    4,    4],
                                        [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8],
                                        [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40] ])

        # diagonal matrix
        self.diag_state_val_Q = self.state_val_Q.flatten(order='F')
        self.diag_state_val_Q = np.diag( self.diag_state_val_Q)


    def mpc_solver(self, states, o_state, pred_con_val, ref_states, con_R, state_Q, dt):
        self.get_logger().info("Solving the MPC controller")

        while (True):
            control_val, state_value = QPC.QPC_solutions( states, o_state, pred_con_val, ref_states, con_R, state_Q, dt)
            if ( np.isnan( control_val[0][0])) :
                print( control_val[0][0], type( control_val[0][0]))
            else :
                break

        return control_val, state_value


    

    def collision_test(self, states, object_states):
        collision_list = []
        for object_num in range( len(object_states[0])):
            x_coord = object_states[0,object_num]
            y_coord = object_states[1,object_num]
            critical_step = np.empty((0,2))
            for state_num in range( len(states[0])):
                distance = ( (states[0,state_num] - x_coord)**2 + (states[1,state_num] - y_coord)**2)**(1/2)
                if ( distance < 0.1):
                    critical_step = np.vstack( (critical_step, np.array([state_num, distance]) ))
            collision_list.append(critical_step)

        return collision_list
    

    def update_states(self, init_state, object_states, ref_states, val_Q):

        collision_list = self.collision_test( ref_states, object_states)
        print( collision_list)

        dir_list = [-1, 1]

        for i in range( len(object_states[0])):
            if ( len(collision_list[i]) > 0):
                # update init_state
                init_state = np.vstack( (init_state, np.array([ [object_states[2][i]],[object_states[1][i]]]) ))

                # update state matrix
                states_update = np.zeros((2,17), dtype=np.float64)
                
                # update Q matrix
                val_Q_update = np.zeros((2,16))
                
                for num in collision_list[i]:
                    dir = dir_list[i]
                    states_update[0][ int(num[0])] = 0.1
                    states_update[1][ int(num[0])] = dir*(0.1 - num[1]/2)

                    val_Q[1][ int(num[0])-1] = 500
                    val_Q_update[0][ int(num[0])-1] = 100
                    val_Q_update[1][ int(num[0])-1] = 100

                ref_states = np.vstack(( ref_states, states_update))
                val_Q = np.vstack(( val_Q, val_Q_update))

        print(init_state)
        print(ref_states)
        print(val_Q)

        val_Q = val_Q.flatten(order='F')
        val_Q = np.diag( val_Q)

        return init_state, ref_states, val_Q
        


    def timer_callback(self):

        init_state_Robot, ref_state_val_Robot = Fc.Convert_To_Robot_Frame( self.X_0.copy(), self.ref_state_val.copy())
        objStat_Robot = Fc.Convert_Obj_To_Robot_Frame( self.X_0.copy(), self.staticObj.copy())

        init_state_update, ref_states_update, val_Q_update = self.update_states(    init_state_Robot.copy(), 
                                                                                    objStat_Robot.copy(), 
                                                                                    ref_state_val_Robot.copy(), self.state_val_Q.copy())

        # mpc solver
        control_val, state_val = self.mpc_solver(   len(init_state_update), 
                                                    init_state_update.copy(), 
                                                    self.pred_control_val.copy(), 
                                                    ref_states_update.copy(), 
                                                    self.control_val_R.copy(), 
                                                    val_Q_update.copy(), 
                                                    self.dt )



        # publish nodes
        self.refMarker_pub.publish( self.reference_markers( ref_state_val_Robot ))
        self.predictMarker_pub.publish( self.predicted_markers( state_val ))
        self.staticObj_pub.publish( self.object_marker( objStat_Robot ))



    def reference_markers(self, ref_val):
        marker_array = MarkerArray()
        for i in range(len(ref_val[0])):
            marker = self.make_marker(i, ref_val[0][i], ref_val[1][i], ref_val[2][i], [1.0, 0.0, 0.0], 0.1)
            marker_array.markers.append(marker)
            # Create the text marker to display step number
            text_marker = self.make_textmarker( (i+500), ref_val[0][i], ref_val[1][i], 0.1, [1.0, 1.0, 1.0], str(i + 1))
            marker_array.markers.append(text_marker)
        return marker_array


    def predicted_markers(self, pre_val):
        marker_array = MarkerArray()
        for i in range(len(pre_val[0])):
            marker = self.make_marker(1000 + i, pre_val[0][i], pre_val[1][i], pre_val[2][i], [0.0, 1.0, 0.0], 0.1)
            marker_array.markers.append(marker)
            # Create the text marker to display step number
            text_marker = self.make_textmarker( (i+1500), pre_val[0][i], pre_val[1][i], 0.1, [1.0, 1.0, 1.0], str(i + 1))
            marker_array.markers.append(text_marker)
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
        marker.header.frame_id = "robot_base"
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
        marker.header.frame_id = "robot_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_obj"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 0.1

        marker.color.r, marker.color.g, marker.color.b = color_rgb
        marker.color.a = 0.5

        marker.pose.position.x = float( x *10)
        marker.pose.position.y = float( y *10)
        marker.pose.position.z = 0.1

        return marker


    def make_textmarker(self, marker_id, x, y, z, color_rgb, text):
            marker = Marker()
            marker.header.frame_id = "robot_base"
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
