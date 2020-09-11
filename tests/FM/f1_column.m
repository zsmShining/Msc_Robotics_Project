clc;clear;close all;
%% set up the formation--the leader
% leader moves in a straight line: heading to the goal
leader_state = [0,0,pi/2,1,pi/20];
% leader moves circularly : designed trajectory
% leader_state = [0,0,pi/2,1,pi/10];
leader_goal = [-10,10];
separation_distance = 4;
bearing_angle = pi/4;
fleet_color = ["r","g","m"];
fleet = formation_lib.createFormation("column",leader_state,leader_goal, ...
                                      separation_distance,bearing_angle);
%fleet trjectory track
fleet_track = [];
%% setup the follower
fo1 = agent(2,"follower",[4,-4,pi/2],[0,0]);
fo2 = agent(3,"follower",[-4,-4,pi/2],[0,0]);
f1ErrorTrack = [];
f2ErrorTrack = [];
follower_track = [];
fo1StateTrack = [];
fo2StateTrack = [];
% Tool_lib.plotScene(fleet.agent,fleet_color);
%% setup simulation
dt = 0.1;
simStep = 400;
%% setup controller1

KMatrix = [2,1,0.01;2.0,0.5,0.01;3.0,0.5,0.01];Boundary = [0,4;-1,1;-1,1];
Lamda1 = [1,2];
ctrF1 = formation_controller(KMatrix,Boundary,Lamda1,dt);

%% setup controller1
KMatrix2 = [1,1,0.01;2.0,0.5,0.01;3.0,0.5,0.01];Boundary = [0,4;-1,1;-1,1];
Lamda2 = [1,1];
ctrF2 = formation_controller(KMatrix2,Boundary,Lamda2,dt);

for t = 0:simStep
%     Tool_lib.plotScene(fleet.agent,fleet_color);
%     Tool_lib.plotScene({fo1,fo2},['g','g']);
%     drawnow;
%     if t~= simStep
%         clf();
%     else
%         break;
%     end
    leader = fleet.agent{1};
    %% --> leader behaviour
    leader = leader.nextStep(dt);
    %% --> follower1 behaviour
    % formation maintainence
    virAgt1 = fleet.agent{2};
    fE1 = fo1.calPoseErrorinLocalFrame(virAgt1);
    f1ErrorTrack = [f1ErrorTrack;fE1'];
    ctrF1 = ctrF1.oneStepControl(fE1);
    ctrlF1Command = ctrF1.output;
    vel1_c = ctrlF1Command(1);omega1_c = ctrlF1Command(2);
    % follower1 take in command to move
    fo1 = fo1.set("linearSpeed",vel1_c);
    fo1 = fo1.set("angularSpeed",omega1_c);
    fo1 = fo1.nextStep(dt);
    %% --> follower2 behaviour
    virAgt2 = fleet.agent{3};
    fE2 = fo2.calPoseErrorinLocalFrame(virAgt2);
    f2ErrorTrack = [f2ErrorTrack;fE2'];
    ctrF2 = ctrF2.oneStepControl(fE2);
    ctrlF2Command = ctrF2.output;
    vel2_c = ctrlF2Command(1);omega2_c = ctrlF2Command(2);
    % follower1 take in command to move
    fo2 = fo2.set("linearSpeed",vel2_c);
    fo2 = fo2.set("angularSpeed",omega2_c);
    fo2 = fo2.nextStep(dt);
    % update fleet--> ideal formation
    fleet_track = [fleet_track;leader.position,virAgt1.position,virAgt2.position];
    follower_track = [follower_track;fo1.position,fo2.position];
    fo1StateTrack = [fo1StateTrack;fo1.get("state")];
    fo2StateTrack = [fo2StateTrack;fo2.get("state")];
    fleet = formation_lib.update(fleet,leader);
end

figure
%trajectory
hold on
grid on 
plot(fleet_track(:,1),fleet_track(:,2),"r");
plot(fleet_track(:,3),fleet_track(:,4),"g--");
plot(fleet_track(:,5),fleet_track(:,6),"b--");
plot(follower_track(:,1),follower_track(:,2),"g");
plot(follower_track(:,3),follower_track(:,4),"b");

axis square

figure
%error
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

figure
grid on
subplot(2,1,1)
hold on
plot(1:length(fo1StateTrack(:,1)),fo1StateTrack(:,4));
plot(1:length(fo1StateTrack(:,2)),fo1StateTrack(:,5));

subplot(2,1,2)
hold on
plot(1:length(fo2StateTrack(:,1)),fo2StateTrack(:,4));
plot(1:length(fo2StateTrack(:,2)),fo2StateTrack(:,5));



