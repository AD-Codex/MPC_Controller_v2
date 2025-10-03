# Method 1

 ---------------------- state equations ----------------------------------------

                         k+1 step
                        .
                     .
                  .
               .
            .
         .
       K step - (X_k, Y_k, theta_k) 
    ( v_k, w_k)

     x_k+1 = x_k + v_k.cos(theta_k).dt
     y_k+1 = y_k + v_k.sin(theta_k).dt
     theta_k+1 = theta_k + w_k.dt
    
     | X_k+1     |   | 1  0  0 | | X_k     |     | cos(theta_k).dt   0 | | V_k |
     | Y_k+1     | = | 0  1  0 |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
     | theta_k+1 |   | 0  0  1 | | theta_k |     |               0  dt | 


 ----------------------- consider a static object ---------------------------------

                        static object - ( X_s, Y_s)
                        ..
                      . .
                    .  .
                  .   .
                .    .
         l_k  .     . l_k+1
            .      . 
          .        k+1 step - (X_k+1, Y_k+1, theta_k+1)             
        . 
     K step - (X_k, Y_k, theta_k)      
       (v_k, w_k)

     orientation between,     K+1 step & K step  =  theta_k
     angle to static object,       alpha_k       =  aniSin[ ly_k/l_k]
     ori & static object angle,                  =  alphs_k - theta_k
                                         l_k+1   =  l_k  -  v_k. cos( alpha_k - theta_k) . dt    ; consider small dts
    
                                         
     x_k+1     = x_k + v_k.cos(theta_k).dt
     y_k+1     = y_k + v_k.sin(theta_k).dt
     theta_k+1 = theta_k + w_k.dt
     l_k+1     = l_k  -  v_k.cos( alpha_k - theta_k).dt
     ly_k+1    = ly_k - v_k.sin(theta_k).dt
    
     | X_k+1     |   | 1  0  0  0  0 | | X_k     |     |            cos(theta_k).dt   0 | | V_k |
     | Y_k+1     | = | 0  1  0  0  0 |.| Y_k     |  +  |            sin(theta_k).dt   0 |.| w_k |
     | theta_k+1 |   | 0  0  1  0  0 | | theta_k |     |                          0  dt | 
     | l_k+1     | = | 0  0  0  1  0 | | l_k     |     |-cos( alpha_k - theta_k).dt   0 |
     | ly_k+1    | = | 0  0  0  0  1 | | ly_k    |     |           -sin(theta_k).dt   0 |


 ----------------- linearize state equations ------------------------------------
 
     x_k+1     = x_k     +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
     y_k+1     = y_k     +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
     theta_k+1 = theta_k +  w_k.dt
     l_k+1     = l_k     -  v_k.cos(alpha_k - theta_k).dt  - v_k.sin(alpha_k - theta_k).dt.theta_k - [v_k.sin(alpha_k-theta_k).dt].l_k/[cos(alpha_k).l_k.l_k] + [v_k.sin(alpha_k-theta_k).dt].ly_k/cos(alpha_k)
     ly_k+1    = ly_k    -  v_k.sin(theta_k).dt  -  v_k.cos(theta_k).dt.theta_k
    
    
     | X_k+1     |   | 1  0           -v_k.sin(theta_k).dt                                                            0                                                  0 | | X_k     |     |            cos(theta_k).dt   0 | | V_k |
     | Y_k+1     | = | 0  1            v_k.cos(theta_k).dt                                                            0                                                  0 |.| Y_k     |  +  |            sin(theta_k).dt   0 |.| w_k |
     | theta_k+1 |   | 0  0                              1                                                            0                                                  0 | | theta_k |     |                          0  dt | 
     | l_k+1     | = | 0  0  - v_k.sin(alpha_k-theta_k).dt  1-[v_k.sin(alpha_k-theta_k).ly_k.dt]/[cos(alpha_k).l_k.l_k]  [v_k.sin(alpha_k-theta_k).dt]/[cos(alpha_k).ly_k] | | l_k     |     |-cos( alpha_k - theta_k).dt   0 |
     | ly_k+1    | = | 0  0           -v_k.cos(theta_k).dt                                                            0                                                  1 | | ly_k    |     |           -sin(theta_k).dt   0 |
    
         X_1    =           A_0                 . X_0         +          B_0            . U_0
         X_2    =           A_1                 . X_1         +          B_1            . U_1




# Method 2

---------------------- state equations ----------------------------------------

                       k+1 step
                      .
                   .
                .
             .
          .
       .
     K step - (X_k, Y_k, theta_k) 
    ( v_k, w_k)

     x_k+1 = x_k + v_k.cos(theta_k).dt
     y_k+1 = y_k + v_k.sin(theta_k).dt
     theta_k+1 = theta_k + w_k.dt

     | X_k+1     |   | 1  0  0 | | X_k     |     | cos(theta_k).dt   0 | | V_k |
     | Y_k+1     | = | 0  1  0 |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
     | theta_k+1 |   | 0  0  1 | | theta_k |     |               0  dt | 


 ----------------------- consider a dynanic object ---------------------------------
 
    object move toward to robot

                                (Xs_k, Ys_k)
         (Xs_k+1, Ys_k+1)      .
                       .    .
                      .  . 
                     . .
                   ..
                .  .
     l_k     .    . l_k+1
          .      . 
       .           k+1 step - (X_k+1, Y_k+1, theta_k+1)             
    .     
     K step - (X_k, Y_k, theta_k)      

     robot movement    ( v_k, w_k)
     object movement   ( X_dot_k, Y_dot_k), positive if move toward robot
    
     robot to object distance,     l_(k+1)_x   =   l_k_x - X_dot_k.dt - v_k.cos(theta_k).dt
                                   l_(k+1)_y   =   l_k_y - Y_dot_k.dt - v_k.sin(theta_k).dt
    
                                         
     x_k+1     = x_k + v_k.cos(theta_k).dt
     y_k+1     = y_k + v_k.sin(theta_k).dt
     theta_k+1 = theta_k + w_k.dt
     l_(k+1)_x = l_k_x - v_k.cos(theta_k).dt - X_dot_k.dt
     l_(k+1)_y = l_k_y - v_k.sin(theta_k).dt - Y_dot_k.dt 

     | X_k+1     |   | 1  0  0  0  0 | | X_k     |     |  cos(theta_k).dt   0 | | V_k |     |   0    0 | | X_dot_k |
     | Y_k+1     | = | 0  1  0  0  0 |.| Y_k     |  +  |  sin(theta_k).dt   0 |.| w_k |  +  |   0    0 |.| Y_dot_k |
     | theta_k+1 |   | 0  0  1  0  0 | | theta_k |     |                0  dt |             |   0    0 |
     | l_(k+1)_x | = | 0  0  0  1  0 | | l_k_x   |     | -cos(theta_k).dt   0 |             | -dt    0 |
     | l_(k+1)_y | = | 0  0  0  0  1 | | l_k_y   |     | -sin(theta_k).dt   0 |             |   0  -dt |


 ----------------- linearize state equations ------------------------------------
 
      x_k+1     = x_k     +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
      y_k+1     = y_k     +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
      theta_k+1 = theta_k +  w_k.dt
      l_(k+1)_x = l_k     -  v_k.cos(theta_k).dt  +  v_k.sin(theta_k).dt.theta_k - X_dot_k.dt
      l_(k+1)_y = ly_k    -  v_k.sin(theta_k).dt  -  v_k.cos(theta_k).dt.theta_k - Y_dot_k.dt 


     | X_k+1     |   | 1  0  -v_k.sin(theta_k).dt  0   0 | | X_k     |     |  cos(theta_k).dt   0 | | V_k |     |   0    0 | | X_dot_k |
     | Y_k+1     | = | 0  1   v_k.cos(theta_k).dt  0   0 |.| Y_k     |  +  |  sin(theta_k).dt   0 |.| w_k |  +  |   0    0 |.| Y_dot_k |
     | theta_k+1 |   | 0  0                     1  0   0 | | theta_k |     |                0  dt |             |   0    0 |
     | l_(k+1)_x | = | 0  0   v_k.sin(theta_k).dt  1   0 | | l_k_x   |     | -cos(theta_k).dt   0 |             | -dt    0 |
     | l_(k+1)_y | = | 0  0  -v_k.cos(theta_k).dt  0   1 | | l_k_y   |     | -sin(theta_k).dt   0 |             |   0  -dt |


     X_1       =                                 A_0  .  X_0       +                      B_0  .  U_0    +         D_0  .  W_0
     X_2       =                                 A_1  .  X_1       +                      B_1  .  U_1    +         D_1  .  W_1








 ----------------------- STATE MATRIX ------------------------------------------
 
     X_predict = phi . x_0 + tau .U_predict +  eta .W_value

     X_predict = [ x_1, y_1, theta_1, l_1, ly_1, x_2, y_2, theta_2, l_2, ly_2 x_3, y_3, theta_3, l_3, ly_3 x_4, y_4, theta_4 l_4, ly_4] ^ T
    
     phi = [          A0]
           [       A1.A0]
           [    A2.A1.A0]
           [ A3.A2.A1.A0]
    
     tau = [          B0,         0,       0,      0]
           [       A1.B0,        B1,       0,      0]    
           [    A2.A1.B0,     A2.B1,      B2,      0]  
           [ A3.A2.A1.B0,  A3.A2.B1,   A3.B2,     B3]
    
     eta = [          D0,         0,       0,      0]
           [       A1.D0,        D1,       0,      0]    
           [    A2.A1.D0,     A2.D1,      D2,      0]  
           [ A3.A2.A1.D0,  A3.A2.D1,   A3.D2,     D3]


 --------------------------- Cost Fn --------------------------------------
 
     J = (X_predict - X_ref)^T . Q . (X_predict - X_ref) + U_predict^T . R . U_predict
       = (1/2) . U_predict^T . H . U_predict + f^T . U_predict + constant
    
     H = tau^T . Q . tau + R
     f = tau^T . Q . ( phi . x_0 + eta . W_value - X_ref)


 -------------------------- daqp general method ------------------------------
 
     (xstar,fval,exitflag,info) = daqp.solve(H,f,A,bupper,blower,sense)

<img width="1919" height="1003" alt="static_1" src="https://github.com/user-attachments/assets/9a40f434-e072-45e3-a831-70f78a8b517c" />

<img width="1919" height="1003" alt="static_2" src="https://github.com/user-attachments/assets/02230999-b0c1-4211-9660-79c928848592" />

<img width="1919" height="1014" alt="D1" src="https://github.com/user-attachments/assets/0c45bdc9-1e44-4e8e-9c51-00c3a7e2fdb3" />

<img width="1919" height="1014" alt="D2" src="https://github.com/user-attachments/assets/e3da4118-6b0a-4903-ab15-b940a22bfdfd" />

