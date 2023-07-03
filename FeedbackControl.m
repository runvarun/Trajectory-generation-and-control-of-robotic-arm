function [V,theta_dot,V_err,V_d]=FeedbackControl(thetalist,Tse_d,Tse_d_next,kp,ki,dt)

global err_count;

S=[[0, 0, 1, -300, 0, 0];
   [0, 1, 0, -240, 0, 0];
   [0, 1, 0, -240, 0, 244];
   [0, 1, 0, -240, 0, 457];
   [0, 0, -1, 169, 457, 0];
   [0, 1, 0, -155, 0, 457]]';

blist=[[0,0,1,-378,457,0];
       [0,1,0,-85,0,-457];
       [0,1,0,-85,0,-213];
       [0,1,0,-85,0,0];
       [0,0,-1,247,0,0];
       [0,1,0,0,0,0]]';

M = [[1, 0, 0, 457]; [0, 1, 0, 78]; [0, 0, 1, 155]; [0, 0, 0, 1]];
  
Tse_i = FKinSpace(M, S, thetalist');
V_err = se3ToVec(MatrixLog6(TransInv(Tse_i) * Tse_d));
err_count=err_count+V_err*dt;
V_d = se3ToVec(MatrixLog6(TransInv(Tse_d) * Tse_d_next)/dt);
AdT = Adjoint(TransInv(Tse_i) * Tse_d);
AdT_V_d = AdT*V_d;
V= AdT_V_d + kp*eye(6)*V_err + ki*eye(6)*err_count;

Jb=JacobianBody(blist, thetalist');
theta_dot = pinv(Jb)*V;

end