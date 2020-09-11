clc;clear;close all;
%% set up the formation--the leader
% leader moves in a straight line: heading to the goal
leader_state = [0,0,pi/2,1,0];
% leader moves circularly : designed trajectory
% leader_state = [0,0,pi/2,1,pi/10];
leader_goal = [-10,10];
separation_distance = 2;
bearing_angle = pi/4;
fleet_color = ["r","g","m"];
fleet = formation_lib.createFormation("line",leader_state,leader_goal, ...
                                      separation_distance,bearing_angle);
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
%% setup simulation
dt = 0.1;
simStep = 400;
%% setup goal controller
Kp = 0.6;
%% setup Formation Controller1
KMatrix = [4,1,0.01;2,0.5,0.01;2,0.5,0.01];Boundary = [0,4;-1,1;-1,1];
Lamda1 = [1.5,1.5];
ctrF1 = formation_controller(KMatrix,Boundary,Lamda1,dt);

%% setup Formation Controller2
%KMatrix2 = [4,1,0.01;2,0.5,0.01;2,0.5,0.01];
KMatrix2 = [4,1,0.01;1,1,0.1;2,0.5,0.01];Boundary = [0,3;-6,6;-6,6];
Lamda2 = [1,1];
ctrF2 = formation_controller(KMatrix2,Boundary,Lamda2,dt);
position = [0,0;leader_goal];
leader = fleet.agent{1};
temp = leader;
trackPos = [leader.position,leader.orientation];
trackDot = [leader.linearSpeed,leader.angularSpeed];
diff = leader_goal - leader.position;
ang = atan2(diff(2),diff(1));
trackAng = [];
trackError = [];
for t = 0:simStep
    hold on
    plot(trackPos(:,1),trackPos(:,2),'r');
    plot(position(:,1),position(:,2),'--b');
    scatter(0,0,"m*")
    Tool_lib.plotScene({fleet.agent{1}},fleet_color);
    drawnow;  
    trackPos = [trackPos;leader.position,leader.orientation];
    trackAng = [trackAng;ang];
    if leader.checkIsAtGoal()
        break;
    elseif t~= simStep
        clf();
    else
        break;
    end
    leader = fleet.agent{1};
    %% --> leader behaviour
    goalError = calGoalError(leader);
    omega_c = Kp*goalError;
    leader = leader.set("angularSpeed",omega_c);
    leader = leader.nextStep(dt);
    fleet = formation_lib.update(fleet,leader);
    diff = leader_goal - leader.position;
    ang = atan2(diff(2),diff(1));
    trackDot = [trackDot;leader.linearSpeed,leader.angularSpeed];
    trackError = [trackError;goalError];
end
Tool_lib.plotScene({temp},"r");
legend("Actual Trajectory","Desired Trajectory");

figure
subplot(2,1,1)
hold on
plot((1:length(trackPos(:,3)))*dt,trackPos(:,3));
plot((1:length(trackAng))*dt,trackAng);
plot((1:length(trackError))*dt,trackError);
legend("Desired Orientation","Actual Orientation","Error")
subplot(2,1,2)
hold on
plot((1:length(trackDot(:,2)))*dt,trackDot(:,2))
legend("angularSpeed")
% figure
% %trajectory
% hold on
% % grid on 
% % plot(fleet_track(:,1),fleet_track(:,2),"r");
% % plot(fleet_track(:,3),fleet_track(:,4),"g--");
% % plot(fleet_track(:,5),fleet_track(:,6),"b--");
% % plot(follower_track(:,1),follower_track(:,2),"k");
% % plot(follower_track(:,3),follower_track(:,4),"k");
% scatter(leader_goal(:,1),leader_goal(:,2),"r*")
% axis square


%error
% figure
% grid on 
% subplot(2,1,1)
% hold on
% plot(1:length(f1ErrorTrack(:,1)),f1ErrorTrack(:,1));
% plot(1:length(f1ErrorTrack(:,2)),f1ErrorTrack(:,2));
% plot(1:length(f1ErrorTrack(:,3)),f1ErrorTrack(:,3));
% subplot(2,1,2)
% hold on
% plot(1:length(f2ErrorTrack(:,1)),f2ErrorTrack(:,1));
% plot(1:length(f2ErrorTrack(:,2)),f2ErrorTrack(:,2));
% plot(1:length(f2ErrorTrack(:,3)),f2ErrorTrack(:,3));
% 
% figure
% grid on
% subplot(2,1,1)
% hold on
% plot(1:length(fo1StateTrack(:,1)),fo1StateTrack(:,4));
% plot(1:length(fo1StateTrack(:,2)),fo1StateTrack(:,5));
% 
% subplot(2,1,2)
% hold on
% plot(1:length(fo2StateTrack(:,1)),fo2StateTrack(:,4));
% plot(1:length(fo2StateTrack(:,2)),fo2StateTrack(:,5));



