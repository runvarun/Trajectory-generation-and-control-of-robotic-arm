clf
clf

blist=[[0,0,1,-378,457,0];
       [0,1,0,-85,0,-457];
       [0,1,0,-85,0,-213];
       [0,1,0,-85,0,0];
       [0,0,-1,247,0,0];
       [0,1,0,0,0,0]]';


Slist=[[0, 0, 1, -300, 0, 0];
   [0, 1, 0, -240, 0, 0];
   [0, 1, 0, -240, 0, 244];
   [0, 1, 0, -240, 0, 457];
   [0, 0, -1, 169, 457, 0];
   [0, 1, 0, -155, 0, 457]]';

M = [[1, 0, 0, 457]; [0, 1, 0, 78]; [0, 0, 1, 155]; [0, 0, 0, 1]];

Thetalist_set_no_err=[-0.5236, -1.5708, 1.5708, -1.5708, -1.5708, 2.6180];
Tse_i = FKinSpace(M, Slist, Thetalist_set_no_err');
Thetalist_set_err_HJ=[-0.8062, -1.5460, 1.6274, -1.2574, -1.2173, 2.2640];


%% TrajectoryGenerator call

Tse_init_no_err=[0,0,1,323.6;
     -1,0,0,-335.6;
     0,-1,0,237;
     0,0,0,1];

Tse_init_err=[0,1,0,247;
          1,0,0,-169;
          0,0,-1,782;
          0,0,0,1];

Tsc_init=[1,0,0,450;
          0,1,0,-300;
          0,0,1,20;
          0,0,0,1];

Tsc_final=[0,-1,0,0;
          1,0,0,100;
          0,0,1,20;
          0,0,0,1];
dt=0.01;

[TRAJECTORIES,traj,T]=TrajectoryGenerator(Tse_init_no_err, Tsc_init, Tsc_final, dt);
gripper_state=TRAJECTORIES(:,13);
% writematrix(TRAJECTORIES,'trajectorycheck.csv')

% %% NextState call
% 
% thetalist=[-0.5236,-1.5708,1.5708,-1.5708,-1.5708,2.6180];
% dthetalist=[0.1,0.2,0.3,0.4,0.5,0.6];
% maxjointvel=[pi,pi,pi,2*pi,2*pi,2*pi];
% dt=0.01;
% T=14;
% output=[];
% for i=1:T/dt
%     thetalistNext = NextState(thetalist, dthetalist, dt, maxjointvel);
%     output=[output;thetalistNext];
%     thetalist=thetalistNext;
% end
% 
% OUTPUT=[output gripper_state];
% % writematrix(OUTPUT,'NextStateOutput.csv')

%% FeedbackControl Call

global err_count
err_count=zeros(6,1);

kp=0;ki=0;
dt=0.01;

theta_test=[-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];

% T_se_d=[0,0,1,300;
%         -1,0,0,-300;
%         0,-1,0,237;
%         0,0,0,1];
% 
% T_se_d_next=[0,0,1,290;
%         -1,0,0,-290;
%         0,-1,0,237;
%         0,0,0,1];
% 
% kp=1;ki=0;
% dt=0.01;
% T=14;
% 
% [V,theta_dot,V_err,V_d]=FeedbackControl(theta_test,T_se_d,T_se_d_next,kp,ki,dt);
% V
% V_err
% V_d

V_ERR=[];

Thetalist_set_no_err=[-0.5236, -1.5708, 1.5708, -1.5708, -1.5708, 2.6180];
Thetalist_set_err=[0.0014, -1.5699, -0.0028, -1.5869, -1.5694, 0];

thetalist=Thetalist_set_err_HJ;
maxjointvel=[pi,pi,pi,2*pi,2*pi,2*pi];
output=[thetalist];

for i=1:(T/dt)-1

    Tse_d=traj{i};
    Tse_d_next=traj{i+1};
    [V,theta_dot,V_err]=FeedbackControl(thetalist,Tse_d,Tse_d_next,kp,ki,dt);
    thetalistNext = NextState(thetalist, theta_dot', dt, maxjointvel);

    % Checking joint limits
    for j=1:6
        if (thetalistNext(j)>3.14)
            thetalistNext(j)=thetalistNext(j)-2*pi;
        end
        if (thetalistNext(j)<-3.14)
            thetalistNext(j)=thetalistNext(j)+2*pi;
        end
    end

    output=[output;thetalistNext];
    thetalist=thetalistNext;
    V_ERR=[V_ERR;V_err'];

end

OUTPUT=[output gripper_state];
writematrix(OUTPUT,'trajectoryOutput_err.csv')

%% Plotting the error twist

for i = 1:length(V_ERR)
    Vb_error = V_ERR(i, :);
    angular_error(i) = round(sqrt( Vb_error(1)^2 + Vb_error(2)^2 + Vb_error(3)^2) , 14) ;
    linear_error(i) = round(sqrt( Vb_error(4)^2 + Vb_error(5)^2 + Vb_error(6)^2) ,3) ;
end

time_axis = dt:dt:(T-dt);

figure(1)
subplot(2, 1, 1)
hold on
title(sprintf('Angular Errors for Kp = %f, Ki = %f', kp, ki))
xlabel('time in s'); ylabel('Error')
ylabel('Angular error')
% xline(2, '--'); xline(4, '--'); xline(6, '--'); xline(10, '--'); xline(12, '--');
% yline(0,'r--', 'LineWidth',2 )
plot(time_axis, angular_error, 'LineWidth',2 )
subplot(2, 1, 2)
hold on
title(sprintf('Linear Error for Kp = %f, Ki = %f', kp, ki))
xlabel('time in s');
ylabel('Linear error in mm/s')
% yline(0,'r--', 'LineWidth',2 )
% xline(2, '--'); xline(4, '--'); xline(6, '--'); xline(10, '--'); xline(12, '--');
plot(time_axis, linear_error, 'LineWidth',2)

%% Individual Error Component Plot
for i = 1:length(V_ERR)
    Vb_error = V_ERR(i, :);
    wx(i) = Vb_error(1);
    wy(i) = Vb_error(2);
    wz(i) = Vb_error(3);
    vx(i) = Vb_error(4);
    vy(i) = Vb_error(5);
    vz(i) = Vb_error(6);
end

figure(2)
subplot(2, 1, 1)
hold on
title(sprintf('Angular Component Error for Kp = %f, Ki = %f', kp, ki))
xlabel('time in sec'); ylabel('error')
ylabel('angular error in rad/s')

plot(time_axis, wx, 'LineWidth',2 )
plot(time_axis, wy, 'LineWidth',2 )
plot(time_axis, wz, 'LineWidth',2 )
legend('wx', 'wy', 'wz')
% xline(2, '--'); xline(4, '--'); xline(6, '--'); xline(10, '--'); xline(12, '--');
% yline(0,'r--', 'LineWidth',2 )


subplot(2, 1, 2)
hold on
title(sprintf('Linear Component Error for Kp = %f, Ki = %f', kp, ki))
xlabel('time in sec'); ylabel('error')
ylabel('linear error in mm/s')

plot(time_axis, vx, 'LineWidth',2 )
plot(time_axis, vy, 'LineWidth',2 )
plot(time_axis, vz, 'LineWidth',2 )
legend('vx', 'vy', 'vz')

% yline(0,'r--', 'LineWidth',2 )
% xline(2, '--'); xline(4, '--'); xline(6, '--'); xline(10, '--'); xline(12, '--');