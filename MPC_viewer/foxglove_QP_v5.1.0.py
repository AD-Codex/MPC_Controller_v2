
# New QPC_dyanmic library with contour controle
# dynamic object avoiding 
# robot coordinate base
# first consider the reference path and get the follower path
# check if the follower path collide with any obstacles

# for simulation calculation,1 step -->  0.025 s , 0.05 m ,   2 m/s
# for real time calculation, 1 step -->      1 s ,  0.5 m , 0.5 m/s


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


class obstacle():
    def __init__(self, position: np.ndarray, velocity: np.ndarray, acceleration: np.ndarray):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.steps = 20
        self.dt = 0.025

        predicted_positions = self.calculate_positions()
        predicted_velocities = self.calculate_velocities()

        self.predicted_positions = predicted_positions
        self.predicted_velocities = predicted_velocities

    def calculate_positions(self) -> np.ndarray:
        t = np.arange(0, self.steps + 1).reshape(1, -1) * self.dt
        return self.position + self.velocity * t + 0.5 * self.acceleration * t**2 

    def calculate_velocities(self) -> np.ndarray:
        t = np.arange(1, self.steps + 1).reshape(1, -1) * self.dt
        return self.velocity + self.acceleration * t 
        

class obstacleList():
    def __init__(self):
        self.steps = 20
        self.obstacles = {}
        self.counter = 1
        self.total_obstacles = 0
        self.obstacles_positions = np.empty( (0, self.steps+1))

    def add_obstacle(self, position, velocity, acceleration):
        obs = obstacle(position, velocity, acceleration)
        self.obstacles[self.counter] = obs

        # Append x and y predictions to all_positions
        self.obstacles_positions    = np.vstack( (self.obstacles_positions, obs.predicted_positions))

        self.counter += 1
        self.total_obstacles += 1
        return self.counter - 1 

    def get_obstacle_count(self):
        return self.total_obstacles




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

        # controller general settings .............................................................................................
        self.pred_horizon = 15
        # step time seconds
        self.dt = 0.025 
        # ------------------------------------------------------------------------------------------------------------------------
        

        # robot states initializing ................................................................................................
        init_state = np.array([[0], [0.1], [0]] , dtype=np.float64)

        # reference values, (numer + 1)
        ref_state  = np.array([ [0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75,  0.8], 
                                [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                                [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0] ] , dtype=np.float64)
        
        # convert reference val to robot frame
        self.robot_init , self.ref_state_val = Fc.Convert_To_Robot_Frame( init_state, ref_state)

        
        # cost fn state value constant (number)
        self.state_val_Q    = np.array( [ [  0.5,  0.5,    1,    1,  1.5,  1.5,    2,    2,  2.5,  2.5,    3,    3,  3.5,  3.5,    4,    4],
                                          [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8],
                                          [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40]] , dtype=np.float64)
        
        
        # predicted control states (number)
        self.pred_control_val   = np.tile( [[1],[0]], 16)
        # self.pred_control_val   = np.tile( [[2],[0]], 16)

        # cost Fn control state constant
        self.control_val_R      = np.zeros( len(self.pred_control_val[0])*2 , dtype=np.float64)
        # self.control_val_R = np.identity( len( self.pred_control_val[0])*2) *0.05


        self.out_state_val      = np.empty((2,0))
        self.out_control_val    = np.empty((2,0))
        # ------------------------------------------------------------------------------------------------------------------------


        # objects in the environment ( robot frame) ...............................................................................
        self.num_obs       = 3

        self.obs_states     = np.array( [ [ [  0.9], [ -0.1]],
                                          [ [  0.7], [  0.3]],
                                          [ [  0.5], [ -0.8]] ], dtype=np.float64)
        
        self.obs_start_vel  = np.array( [ [ [   -2], [    0]],
                                          [ [    0], [    0]],
                                          [ [    0], [   -1]] ], dtype=np.float64)
        
        self.obs_acce       = np.array( [ [ [    0], [    0]],
                                          [ [    0], [    0]],
                                          [ [    0], [  0.1]] ], dtype=np.float64)
        
        self.obs_list = obstacleList()
        for pos, vel, acc in zip(self.obs_states, self.obs_start_vel, self.obs_acce):
            self.obs_list.add_obstacle(pos, vel, acc)
        # ------------------------------------------------------------------------------------------------------------------------

        


    

    def timer_callback(self):

        pred_horizon = 15

        # first consider the reference path and get the follower path
        self.out_control_val, self.out_state_val  = self.mpc_solver(robot_init              = self.robot_init,                              # (3,1)
                                                                    robot_control_val       = self.pred_control_val[ :, :pred_horizon],     # [ :, :pred_horizon]
                                                                    robot_control_Rval      = self.control_val_R[ :pred_horizon*2],         # [ :pred_horizon*2]
                                                                    objs_init               = None,                                         # (2,1)
                                                                    objs_pred_control_val   = None,                                         # [ :, :pred_horizon]
                                                                    objs_path               = None,                                         # [ :, :pred_horizon+1]
                                                                    objs_ref_path_val       = None,                                         # [ :, :pred_horizon+1],
                                                                    objs_ref_path_Qval      = None,                                         # [ :, :pred_horizon]
                                                                    ref_path_val            = self.ref_state_val[ :, :pred_horizon+1],      # [ :, :pred_horizon+1]
                                                                    ref_path_Qval           = self.state_val_Q[ :, :pred_horizon],          # [ :, :pred_horizon]
                                                                    pred_horizon            = pred_horizon  )                               # int
        

        # check if the follower path collide with any obstacles
        objs_init               = np.empty( (0, 1))
        objs_pred_control_val   = np.empty( (0, pred_horizon))
        objs_path               = np.empty( (0, pred_horizon+1))
        objs_ref_path_val       = np.empty( (0, pred_horizon+1))
        objs_ref_path_Qval      = np.empty( (0, pred_horizon))
        objs_avoid_dir          = np.array( [-1,-1, 1, 1])
        objs_avoid_weight       = np.array( [100,100,100, 100])
        i= 0
        for obs in self.obs_list.obstacles.values():
            collide_states  = self.collision_detect( self.out_state_val[ :, :pred_horizon+1], obs.predicted_positions[ :, :pred_horizon+1])

            if collide_states.any() :
                obj_ref_path_val, obj_ref_path_Qval = self.objs_ref_path_Q(pred_horizon, self.out_state_val[ :, :pred_horizon+1], obs.predicted_positions[ :, :pred_horizon+1], collide_states, objs_avoid_dir[i], objs_avoid_weight[i])
                i = i + 1
                print(obj_ref_path_val, obj_ref_path_Qval)
                objs_init               = np.vstack( (objs_init, obs.position))
                objs_pred_control_val   = np.vstack( (objs_pred_control_val, obs.predicted_velocities[ :, :pred_horizon]))
                objs_path               = np.vstack( (objs_path, obs.predicted_positions[ :, :pred_horizon+1]))
                objs_ref_path_val       = np.vstack( (objs_ref_path_val, obj_ref_path_val))
                objs_ref_path_Qval      = np.vstack( (objs_ref_path_Qval, obj_ref_path_Qval))

        if objs_init.any() :
            self.out_control_val, self.out_state_val  = self.mpc_solver(robot_init              = self.robot_init,                              # (3,1)
                                                                        robot_control_val       = self.pred_control_val[ :, :pred_horizon],     # [ :, :pred_horizon]
                                                                        robot_control_Rval      = self.control_val_R[ :pred_horizon*2],         # [ :pred_horizon*2]
                                                                        objs_init               = objs_init,                                    # (2,1)
                                                                        objs_pred_control_val   = objs_pred_control_val,                        # [ :, :pred_horizon]
                                                                        objs_path               = objs_path,                                    # [ :, :pred_horizon+1]
                                                                        objs_ref_path_val       = objs_ref_path_val,                            # [ :, :pred_horizon+1],
                                                                        objs_ref_path_Qval      = objs_ref_path_Qval,                           # [ :, :pred_horizon]
                                                                        ref_path_val            = self.ref_state_val[ :, :pred_horizon+1],      # [ :, :pred_horizon+1]
                                                                        ref_path_Qval           = self.state_val_Q[ :, :pred_horizon],          # [ :, :pred_horizon]
                                                                        pred_horizon            = pred_horizon  )





        
        # publish nodes
        self.predictMarker_pub.publish( self.predicted_markers( self.out_state_val))
        self.refMarker_pub.publish( self.reference_markers( self.ref_state_val ))
        self.staticObj_pub.publish( self.object_marker( self.obs_list.obstacles_positions))





    # collision test function
    def collision_detect(self, follow_path, obj_path):
        space = 0.11
        abs_diff    = np.abs( follow_path[:2,:] - obj_path)
        condition   = ( abs_diff <= space).all(axis=0)
        positions   = np.where(condition)[0]

        return positions


    # update the object state matrix and Q matrix prevent collition
    def objs_ref_path_Q(self, pred_horizon, follow_path, obj_path, collide_states, avoid_dir, avoid_weight):
        space = 0.1
        obj_ref_path_val   = np.zeros( (2, pred_horizon+1), dtype=np.float64)
        obj_ref_path_Qval  = np.zeros( (2, pred_horizon), dtype=np.float64)

        for state in collide_states:
            follow_path_state   = follow_path[ :, state]
            obj_path_state     = obj_path[ :, state]

            obj_ref_path_val[0][state]     = obj_path_state[0] - avoid_dir*space*math.sin(follow_path_state[2]) - follow_path_state[0]
            obj_ref_path_val[1][state]     = avoid_dir*space*math.cos(follow_path_state[2])
            obj_ref_path_Qval[0][state-1]  = avoid_weight
            obj_ref_path_Qval[1][state-1]  = avoid_weight


        return obj_ref_path_val, obj_ref_path_Qval



    # model predictive controller solve
    def mpc_solver(self, robot_init, robot_control_val, robot_control_Rval, objs_init, objs_pred_control_val, objs_path, objs_ref_path_val, objs_ref_path_Qval, ref_path_val, ref_path_Qval, pred_horizon):
            # out_control_val, out_state_val  = self.mpc_solver(  robot_init              = self.robot_init,                              # (3,1)
            #                                                     robot_control_val       = self.pred_control_val[ :, :pred_horizon],     # [ :, :pred_horizon]
            #                                                     robot_control_Rval      = self.control_val_R[ :pred_horizon*2],         # [ :pred_horizon*2]
            #                                                     objs_init               = self.Obj_state,                               # (2,1)
            #                                                     objs_pred_control_val   = self.obj_vel[ :, :pred_horizon],              # [ :, :pred_horizon]
            #                                                     objs_path               = self.obj_path[ :, :pred_horizon+1],           # [ :, :pred_horizon+1]
            #                                                     objs_ref_path_val       = self.obj_ref[ :, :pred_horizon+1],            # [ :, :pred_horizon+1],
            #                                                     objs_ref_path_Qval      = self.obj_valQ[ :, :pred_horizon],             # [ :, :pred_horizon]
            #                                                     ref_path_val            = self.ref_state_val[ :, :pred_horizon+1],      # [ :, :pred_horizon+1]
            #                                                     ref_path_Qval           = self.state_val_Q[ :, :pred_horizon],          # [ :, :pred_horizon]
            #                                                     pred_horizon            = pred_horizon  )
            
        # origin state setup
        if objs_init is None:
            objs_init    = np.empty((0,1))
        O_state         = np.vstack( ( robot_init, objs_init))
        
        # number of states
        num_states      = len(O_state)

        # referenve state values setup
        if objs_ref_path_val is None:
            objs_ref_path_val   = np.empty( (0, pred_horizon+1))
        ref_state_val       = np.vstack( ( ref_path_val, objs_ref_path_val))

        # state Q matrix setup
        if objs_ref_path_Qval is None:
            objs_ref_path_Qval   = np.empty( (0, pred_horizon))
        ref_state_Qval      = np.vstack( ( ref_path_Qval, objs_ref_path_Qval))
        diag_state_val_Q    = ref_state_Qval.flatten(order='F')
        diag_state_val_Q    = np.diag( diag_state_val_Q)

        # object control values on robot frame
        if objs_pred_control_val is None:
            objs_pred_control_val_robotFrame    = np.empty( (0, pred_horizon))
        else:
            objs_pred_control_val_robotFrame    = self.obj_states(  pred_horizon= pred_horizon,
                                                                    ref_state   = ref_path_val.copy(),
                                                                    objs_path   = objs_path.copy(),
                                                                    obj_vel     = objs_pred_control_val.copy() )
            
        
        # mpc contorller
        out_control_val, out_state_val = QPC.QPC_solutions( num_state       = num_states,
                                                            pred_horizon    = pred_horizon,
                                                            init_state      = O_state.copy(),
                                                            pred_control_val= robot_control_val.copy(),
                                                            obj_vel         = objs_pred_control_val_robotFrame.copy(),
                                                            ref_state_val   = ref_state_val.copy(),
                                                            control_val_R   = robot_control_Rval.copy(),
                                                            state_val_Q     = diag_state_val_Q.copy(),
                                                            dt              = self.dt)
                
        return out_control_val, out_state_val


    # update object velocity on robot frame ( 0,0)
    def obj_states(self, pred_horizon, ref_state, objs_path, obj_vel):
        objs        = int(len(objs_path)//2)

        # robot relative object velocity
        # if move to robot velocity positive, else move away velocity negative (x,y coordinate vise)
        objs_vel_rel_robot  = np.empty((0,pred_horizon))
        for obj in range(objs):
            obj_vel_rel_robot = np.empty((2,0))
            for state in range( pred_horizon):
                ref_state_  = ref_state [     0:         2, state:state+1]
                obj_state_  = objs_path[ obj*2: (obj+1)*2, state:state+1]
                obj_vel_    = obj_vel   [ obj*2: (obj+1)*2, state:state+1]

                if ( (obj_state_[0][0]-ref_state_[0][0]) >= 0):
                    obj_vel_[0][0] = obj_vel_[0][0] * -1
                if ( (obj_state_[1][0]-ref_state_[1][0]) >= 0):
                    obj_vel_[1][0] = obj_vel_[1][0] * -1

                # print( ref_state_, obj_state_, obj_vel_)
                obj_vel_rel_robot = np.hstack( ( obj_vel_rel_robot, obj_vel_))
            objs_vel_rel_robot = np.vstack( ( objs_vel_rel_robot, obj_vel_rel_robot))

        # print(objs_vel_rel_robot)

        return objs_vel_rel_robot





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
        # print(obj_states)
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

        marker.scale.x = 2.0
        marker.scale.y = 2.0
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





