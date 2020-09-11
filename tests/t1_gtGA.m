%% leader go to the goal
clc;clear;close all;
%% set up the formation
% leader moves in a straight line: heading to the goal
leader_state = [0,0,pi/2];
% leader moves circularly : designed trajectory
% leader_state = [0,0,pi/2,1,pi/10];
leader_goal = [-10,10];
separation_distance = 4;
bearing_angle = pi/4;
fleet_color = ["r","g","m"];
fleet = formation_lib.createFormation("column",leader_state,leader_goal, ...
                                      separation_distance,bearing_angle);
Tool_lib.plotScene(fleet.agent,fleet_color);

%% setup obstacle
obs1 = obstacle(2,[-5,6,0,0,0],1);
obs_color = ['b'];
%% simulation set up
dt = 0.1;
simStep = 200;
% leader moves to goal

%% parameters for controller
goalKp = 0.6;
for t = 0:simStep
    Tool_lib.plotScene(fleet.agent,fleet_color);
    Tool_lib.plotScene({obs1},obs_color);
    drawnow;
    leader = fleet.agent{1};
    goalError = calGoalError(leader);
    w_c = goalKp * goalError;
    leader = leader.set("angularSpeed",w_c);
    leader = leader.nextStep(dt);
    fleet = formation_lib.update(fleet,leader);
end