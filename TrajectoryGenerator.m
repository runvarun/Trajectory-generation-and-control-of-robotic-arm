function [TRAJECTORIES,traj] = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, dt)

%      Inputs : Tse_init - Initial position of end effector wrt space frame
%               Tsc_init - Initial position of cube effector wrt space frame
%               Tse_init - Final position of cube effector wrt space frame
%               dt       - time step

%      Outputs : traj - Matrix containing position of end effector
%                       at each point in the trajectory 
%                TRAJECTORIES - Matrix that is convereted to csv file 
%                               for the  trajectory_test.m script to read

                   
T_standoff1=[1,0,0,450;  % Defining the standoff position above initial cube position
          0,1,0,-300;
          0,0,1,100;
          0,0,0,1];

T_standoff2=[0,-1,0,0;  % Defining the standoff position above final cube position
          1,0,0,100;
          0,0,1,100;
          0,0,0,1];

gripper_state=[];       % Vector that stores gripper state
TRAJ=[];

% 1st segment (initial e-e position to standoff 1)
Tf1=3;
N1=Tf1/dt;
traj1=ScrewTrajectory(Tse_init,T_standoff1,Tf1,N1,3);
gripper_state(1:N1)=0;

% 2nd segment (standoff 1 to Tsc_init)
Tf2=2;
N2=Tf2/dt;
traj2=ScrewTrajectory(T_standoff1,Tsc_init,Tf2,N2,3);
gripper_state(N1+1:N1+N2)=0;

% 3rd and 4th segments (close gripper and Tsc_init to standoff 1)
Tf3=2;
N3=Tf3/dt;
traj3=ScrewTrajectory(Tsc_init,T_standoff1,Tf3,N3,3);
gripper_state(N1+N2+1:N1+N2+N3)=1;

% 5th segment (standoff 1 to standoff 2)
Tf4=3;
N4=Tf4/dt;
traj4=ScrewTrajectory(T_standoff1,T_standoff2,Tf4,N4,3);
gripper_state(N1+N2+N3+1:N1+N2+N3+N4)=1;

% 6th segment (standoff 2 to Tsc_final)
Tf5=2;
N5=Tf5/dt;
traj5=ScrewTrajectory(T_standoff2,Tsc_final,Tf5,N5,3);
gripper_state(N1+N2+N3+N4+1:N1+N2+N3+N4+N5)=1;

% 7th and 8th segments (Open gripper and Tsc_final to standoff 2) 
Tf6=2;
N6=Tf6/dt;
traj6=ScrewTrajectory(Tsc_final,T_standoff2,Tf6,N6,3);
gripper_state(N1+N2+N3+N4+N5+1:N1+N2+N3+N4+N5+N6)=0;

traj=[traj1,traj2,traj3,traj4,traj5,traj6];

for i=1:1400
    TRAJ = [TRAJ;traj{i}(1,1:3), traj{i}(2,1:3), traj{i}(3,1:3), traj{i}(1:3,4)'];
end

TRAJECTORIES=[TRAJ gripper_state'];
end