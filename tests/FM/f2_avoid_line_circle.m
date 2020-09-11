% Hybrid Control Behaviour-->Just Avoid Obstacle
% adopted from f1_line.m;
% leader.move to goal->w + v is constant ->vector->collision ...
% avoidance->final vector->[v,w] 
% follower.maintain formation->[v,w]->vector-> collision avoidance->
% final vector -> [v,w]
clc;clear;close all;
%% set up the formation--the leader
% leader moves in a straight line: heading to the goal
leader_state = [0,0,pi/2,2,pi/20];
% leader moves circularly : designed trajectory
% leader_state = [0,0,pi/2,1,pi/10];
leader_goal = [-20,20];
separation_distance = 4;
bearing_angle = pi/4;
fleet_color = ["r","g","m"];
fleet = formation_lib.createFormation("vshape",leader_state,leader_goal, ...
                                      separation_distance,bearing_angle);
% boundary for the leader's linearSpeed and angularSpeed
BoundLeader = [Object_lib.agentNomialLinearSpeed,Object_lib.agentMaxAngularSpeed];
BoundFollower = [10,30];
%fleet trjectory track
fleet_track = [];
%% setup the follower
fo1 = agent(2,"follower",[2,-4,pi/2],[0,0]);
fo2 = agent(3,"follower",[-2,-4,pi/2],[0,0]);
f1ErrorTrack = [];
f2ErrorTrack = [];
follower_track = [];
fo1StateTrack = [];
fo2StateTrack = [];
% Tool_lib.plotScene(fleet.agent,fleet_color);
%% setup the obstacle
% static
obs1 = obstacle(4,[-5,4,0,0,0],1);
obs2 = obstacle(5,[-5,10,0,0,0],1);
%% setup simulation
dt = 0.1;
simStep = 400;
%% setup goal controller
Kp = 0.6;
%% setup Formation Controller1
KMatrix = [4,1,0.01;2,0.5,0.01;2,0.5,0.01];
Boundary1 = [0,5;-10,10;-10,10];

Lamda1 = [1.5,1.5];
ctrF1 = formation_controller(KMatrix,Boundary1,Lamda1,dt);

%% setup Formation Controller2
%KMatrix2 = [4,1,0.01;2,0.5,0.01;2,0.5,0.01];
KMatrix2 = [4,0.4,0.1;1.0,0.5,0.1;1,0.5,0.01];

Boundary2 = [0,5;-10,10;-10,10];
Lamda2 = [1.5,2];
ctrF2 = formation_controller(KMatrix2,Boundary2,Lamda2,dt);

for t = 0:simStep
    Tool_lib.plotScene(fleet.agent,fleet_color);
    Tool_lib.plotScene({fo1,fo2},['g','g']);
    Tool_lib.plotScene({obs1,obs2},['m','m']);
    drawnow;
    if t~= simStep
        clf();
    else
        break;
    end
    leader = fleet.agent{1};
    %% --> leader behaviour
    % Leader-Behaviour1 move to goal 
%     goalError = calGoalError(leader);
%     MG_omega_c = Kp*goalError;
%     MG_vel_c = Object_lib.agentNomialLinearSpeed;
%     MGvector = convertCommand(leader,MG_vel_c,MG_omega_c,dt);
    MG_omega_c = leader.angularSpeed;
    MG_vel_c = leader.linearSpeed;
    MGvector = convertCommand(leader,MG_vel_c,MG_omega_c,dt);
    % plug the MGvector into the collision avoidance behaviour
    % Leader-Behaviour2 - colliison avoidance
    ACvector = leader.calAvoidCollision({obs1,obs2},MGvector);
    [AC_vel_c,AC_omega_c] = convertGuideVector(leader,MGvector,dt,BoundLeader);
    % plug in final command
    leader = leader.set("linearSpeed",AC_vel_c);
    leader = leader.set("angularSpeed",AC_omega_c);
    leader = leader.nextStep(dt);
    %% --> follower1 behaviour
    % Follower1-Behaviour1-formation maintainence
    virAgt1 = fleet.agent{2};
    fE1 = fo1.calPoseErrorinLocalFrame(virAgt1);
    f1ErrorTrack = [f1ErrorTrack;fE1'];
    ctrF1 = ctrF1.oneStepControl(fE1);
    ctrlF1Command = ctrF1.output;
    FK_ve11_c = ctrlF1Command(1);FK_omega1_c = ctrlF1Command(2);
    % Follower1-Behaviour2-avoid collision
    FK1vector = convertCommand(fo1,FK_ve11_c,FK_omega1_c,dt);
    f1ACvector = fo1.calAvoidCollision({obs1,obs2},FK1vector);
    [f1AC_vel_c,f1AC_omega_c] = convertGuideVector(fo1,f1ACvector,dt,BoundFollower);
    % follower1 take in command to move
    fo1 = fo1.set("linearSpeed",f1AC_vel_c);
    fo1 = fo1.set("angularSpeed",f1AC_omega_c);
    fo1 = fo1.nextStep(dt);
    %% --> follower2 behaviour
    % Follower2-Behaviour1-formation maintainence
    virAgt2 = fleet.agent{3};
    fE2 = fo2.calPoseErrorinLocalFrame(virAgt2);
    f2ErrorTrack = [f2ErrorTrack;fE2'];
    ctrF2 = ctrF2.oneStepControl(fE2);
    ctrlF2Command = ctrF2.output;
    FK_vel2_c = ctrlF2Command(1);FK_omega2_c = ctrlF2Command(2);
    FK2vector = convertCommand(fo2,FK_vel2_c,FK_omega2_c,dt);
    % Follower2-Behaviour2-avoid collision
    f2ACvector = fo2.calAvoidCollision({obs1,obs2},FK2vector);
    [f2AC_vel_c,f2AC_omega_c] = convertGuideVector(fo2,f2ACvector,dt,BoundFollower);
    % Follower2 take in command to move
    fo2 = fo2.set("linearSpeed",f2AC_vel_c);
    fo2 = fo2.set("angularSpeed",f2AC_omega_c);
    fo2 = fo2.nextStep(dt);
    % update fleet--> ideal formation
    fleet_track = [fleet_track;leader.position,virAgt1.position,virAgt2.position];
    follower_track = [follower_track;fo1.position,fo2.position];
    fo1StateTrack = [fo1StateTrack;fo1.get("state")];
    fo2StateTrack = [fo2StateTrack;fo2.get("state")];
    fleet = formation_lib.update(fleet,leader);
    if leader.checkIsAtGoal()
        break;
    end
end



figure(2)
%trajectory
hold on
grid on 
plot(fleet_track(:,1),fleet_track(:,2),"r");
plot(fleet_track(:,3),fleet_track(:,4),"g--");
plot(fleet_track(:,5),fleet_track(:,6),"b--");
plot(follower_track(:,1),follower_track(:,2),"k");
plot(follower_track(:,3),follower_track(:,4),"k");
scatter(leader_goal(:,1),leader_goal(:,2),"r*")
axis square

%error
figure(3)
grid on 
subplot(2,1,1)
hold on
plot(1:length(f1ErrorTrack(:,1)),f1ErrorTrack(:,1));
plot(1:length(f1ErrorTrack(:,2)),f1ErrorTrack(:,2));
plot(1:length(f1ErrorTrack(:,3)),f1ErrorTrack(:,3));
subplot(2,1,2)
hold on
plot(1:length(f2ErrorTrack(:,1)),f2ErrorTrack(:,1));
plot(1:length(f2ErrorTrack(:,2)),f2ErrorTrack(:,2));
plot(1:length(f2ErrorTrack(:,3)),f2ErrorTrack(:,3));

figure(4)
grid on
subplot(2,1,1)
hold on
plot(1:length(fo1StateTrack(:,1)),fo1StateTrack(:,4));
plot(1:length(fo1StateTrack(:,2)),fo1StateTrack(:,5));

subplot(2,1,2)
hold on
plot(1:length(fo2StateTrack(:,1)),fo2StateTrack(:,4));
plot(1:length(fo2StateTrack(:,2)),fo2StateTrack(:,5));



