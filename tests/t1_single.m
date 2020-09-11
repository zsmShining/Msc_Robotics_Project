clc;clear;close all;
%% set up the formation
leader_state = [0,0,pi/2];
leader_goal = [-20,20];
separation_distance = 4;
bearing_angle = pi/4;
fleet = formation_lib.createFormation("column",leader_state,leader_goal, ...
                                      separation_distance,bearing_angle);
fleet2 = formation_lib.createFormation("column",[0,0,pi/2],leader_goal, ...
                                      separation_distance,bearing_angle);
fleet_color = ["r","g","m"];
%% set up the follower
follower1  = agent(2,"follower",[4,-1,0,pi/2,0],[-10,5]);
follower2 = agent(3,"follower",[4,1,0,pi/2,0],[-10,5]);
followerList = {follower1,follower2};
follow_color = ["g","m"];
%% set up time step size for the simulation
% total duration: simStep*time step size
dt = 0.1;
simStep = 100;
%% set up controller
% controller for moving to the goal
leader_linear_bound = Object_lib.agentNomialLinearSpeed;
leader_angular_bound = 1;
leader_ctrl_bound = [-leader_angular_bound,leader_angular_bound];
K_goal = [0.6,0,0];
ctrlGoalP = PIDcontroller(K_goal,leader_ctrl_bound,dt);
K_goalPID = [0.6,0.5,0.01];
ctrlGoalPID = PIDcontroller(K_goal,leader_ctrl_bound,dt);
%% calculate weight for the move-to-goal behaviour
weightGoal=weightCalculator();
for t = 1:simStep
    % disp the leader and the virtual target in the environment
    Tool_lib.plotScene(fleet.agent,fleet_color);
    Tool_lib.plotScene(fleet2.agent,['k','k','k']);
    drawnow
    if t~= simStep
        clf()
    else
        break;
    end
    %must be bounded within [-pi,pi];
    goalError = calGoalError(fleet.leader);
    % only control the heading angle
    % leader operates at a constant speed where lineaderSpeed = 1.0m/s
    %ctrlGoal = ctrlGoal.oneStepControl(goalError);
    ctrlGoalP = ctrlGoalP.oneStepControl(goalError);
    omega_c = ctrlGoalP.output;
    % input the control command to the agent
    w=weightGoal.forGoal(fleet.leader,dt);
    fleet.leader = fleet.leader.set("linearSpeed",2*w);
    fleet.leader = fleet.leader.set("angularSpeed",omega_c);
    fleet.leader = fleet.leader.nextStep(dt);
    fleet = formation_lib.update(fleet,fleet.leader);    
end



