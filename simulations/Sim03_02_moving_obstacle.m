% Motor Schema--->Taking Average-->Just Avoid Obstacle
% adopted from f1_line.m;
% leader.move to goal->w + v is constant ->vector->collision ...
% avoidance->final vector->[v,w] 
% follower.maintain formation->[v,w]->vector-> collision avoidance->
% final vector -> [v,w]
clc;clear;close all;
%% set up the formation--the leader
% leader moves in a straight line: heading to the goal
leader_state = [0,0,pi/2,2,0];
% leader moves circularly : designed trajectory
% leader_state = [0,0,pi/2,1,pi/10];
leader_goal = [0,35];
separation_distance = 5;
bearing_angle = pi/4;
fleet_color = ["r","g","m"];
fleet = formation_lib.createFormation("line",leader_state,leader_goal, ...
                                      separation_distance,bearing_angle);
% boundary for the leader's linearSpeed and angularSpeed
BoundLeader = [Object_lib.agentNomialLinearSpeed,Object_lib.agentMaxAngularSpeed];
BoundFollower = [5,10];
%fleet trjectory track
fleet_track = [];
leaderStateTrack = [];
Lweight = weightCalculator();
%% setup the follower
fo1 = agent(2,"follower",[-1,-5,pi/2],[0,0]);
fo2 = agent(3,"follower",[-1,-10,pi/2],[0,0]);
F1weight = weightCalculator();
F2weight = weightCalculator();
f1ErrorTrack = [];
f2ErrorTrack = [];
follower_track = [];
fo1StateTrack = [];
fo2StateTrack = [];
% Tool_lib.plotScene(fleet.agent,fleet_color);
%% setup the obstacle
% static
obs1 = obstacle(4,[0,25,-pi/2,1,0],1);
% obs2 = obstacle(5,[0,10,0,0,0],1);
% obs3 = obstacle(6,[5,5,0,0,0],1);
% obs4 = obstacle(6,[0,5,0,0,0],1);
% 
% obs5 = obstacle(7,[3,15,pi,1,0],1);
% obs6 = obstacle(8,[0,30,-3*pi/8,1,0],1);
obsList = {obs1};
obs5Track = [];
obs6Track = [];
%% setup simulation
dt = 0.1;
simStep = 400;
%% setup goal controller
Kp = 0.6;
%% setup Formation Controller1
% KMatrix = [4,0.1,0.1;1.5,0.5,0.01;2,1,0.1];
% Boundary1 = [0,5;-2,2;-1,1];
% Lamda1 = [1,2];

KMatrix = [4,0.1,0.1;2,0.5,0.01;1,1,0.1];
Boundary1 = [-1,5;-10,10;-10,10];
Lamda1 = [2,2];

ctrF1 = formation_controller(KMatrix,Boundary1,Lamda1,dt);

%% setup Formation Controller2
%KMatrix2 = [4,1,0.01;2,0.5,0.01;2,0.5,0.01];
KMatrix2 = [2,0.4,0.1;2.0,0.5,0.1;2.0,1,0.2];

Boundary2 = [-1,5;-10,10;-10,10];
Lamda2 = [1,2];
ctrF2 = formation_controller(KMatrix2,Boundary2,Lamda2,dt);
leader = fleet.agent{1};
objList = {leader,fo1,fo2};
newObjList = objList;
obs1Track = [];
for t = 0:simStep
    leader = fleet.agent{1};
    objList = newObjList;
    List = [obsList,objList];
    if leader.checkIsAtGoal()
        break;
    end
    if t~=400
    if t~= simStep
        clf();
    else
        break;
    end
    else
        break;
    end
    Tool_lib.plotScene(fleet.agent,fleet_color);
    Tool_lib.plotScene({fo1,fo2},['g','b']);
    Tool_lib.plotScene(obsList,['m','m','m','m','c','c']);
    drawnow;


    %% --> leader behaviour
    % Leader-Behaviour1 move to goal 
    goalError = calGoalError(leader);
    MG_omega_c = Kp*goalError;
    MG_vel_c = Object_lib.agentNomialLinearSpeed;
    MGvector = convertCommand(leader,MG_vel_c,MG_omega_c,dt);
    % plug the MGvector into the collision avoidance behaviour
    % Leader-Behaviour2 - colliison avoidance
    objList = [objList,obsList];
    ACvector = leader.calAvoidCollision(List,MGvector);
    lDo = findMinDis(leader,obsList);
%     disp(f1Do)
    Lweight = Lweight.cal(lDo);
    finalVector = (MGvector)*Lweight.weight2+(ACvector)*Lweight.weight1;
%     finalVector = (MGvector)/2+(ACvector)/2;
    [AC_vel_c,AC_omega_c] = convertGuideVector(leader,finalVector,dt,BoundLeader);
    % plug in final command
    leader = leader.set("linearSpeed",AC_vel_c);
    leader = leader.set("angularSpeed",AC_omega_c);
    leader = leader.nextStep(dt);
    newObjList{1} = leader;
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
    f1ACvector = fo1.calAvoidCollision(List,FK1vector);
    f1Do = findMinDis(fo1,obsList);
    F1weight = F1weight.cal(f1Do);
    finalVector = (FK1vector)*F1weight.weight2 + (f1ACvector)*F1weight.weight1;
%     finalVector = (FK1vector)/2+(f1ACvector)/2;
    [f1AC_vel_c,f1AC_omega_c] = convertGuideVector(fo1,finalVector,dt,BoundFollower);
    if f1AC_omega_c > 10
        f1AC_omega_c = 10;
    end
    % follower1 take in command to move
    fo1 = fo1.set("linearSpeed",f1AC_vel_c);
    fo1 = fo1.set("angularSpeed",f1AC_omega_c);
    fo1 = fo1.nextStep(dt);
     newObjList{2} = fo1;
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
    f2ACvector = fo2.calAvoidCollision(List,FK2vector);
    f2Do = findMinDis(fo2,obsList);
    F2weight = F2weight.cal(f2Do);
    disp(F2weight)
    finalVector = (FK2vector)*F2weight.weight2 + (f2ACvector)*F2weight.weight1;
%     finalVector = (FK2vector)/2+(f2ACvector)/2;
    [f2AC_vel_c,f2AC_omega_c] = convertGuideVector(fo2,finalVector,dt,BoundFollower);
    % Follower2 take in command to move
    if f2AC_omega_c > 10
        f2AC_omega_c = 10;
    end
    fo2 = fo2.set("linearSpeed",f2AC_vel_c);
    fo2 = fo2.set("angularSpeed",f2AC_omega_c);
    
    fo2 = fo2.nextStep(dt);
    newObjList{2} = fo1;
    % update fleet--> ideal formation
    fleet_track = [fleet_track;leader.position,virAgt1.position,virAgt2.position];
    follower_track = [follower_track;fo1.position,fo2.position];
    fo1StateTrack = [fo1StateTrack;fo1.get("state")];
    fo2StateTrack = [fo2StateTrack;fo2.get("state")];
    leaderStateTrack = [leaderStateTrack;leader.get("state")];
    fleet = formation_lib.update(fleet,leader);
    obs1 = obs1.nextStep(dt);
    obsList{1} = obs1;
%    obs5 = obs5.nextStep(dt);
%     obs6 = obs6.nextStep(dt);
%    obsList{end-1} = obs5;
%     obsList{end} = obs6;
   obs1Track = [obs1Track;obs1.position];
%     obs6Track = [obs6Track;obs6.position];
end



%trajectory
hold on
plot(fleet_track(:,1),fleet_track(:,2),"r");
plot(fleet_track(:,3),fleet_track(:,4),"g--");
plot(fleet_track(:,5),fleet_track(:,6),"b--");
plot(follower_track(:,1),follower_track(:,2),"g");
plot(follower_track(:,3),follower_track(:,4),"b");
plot(obs1Track(:,1),obs1Track(:,2),"c");
% plot(obs6Track(:,1),obs6Track(:,2),"c");
scatter(leader_goal(:,1),leader_goal(:,2),"r*")


figure
%error
grid on 
subplot(2,1,1)
hold on
% plot(1:length(f1ErrorTrack(:,1)),f1ErrorTrack(:,1));
% plot(1:length(f1ErrorTrack(:,2)),f1ErrorTrack(:,2));
relDisError1 = [];
for i = 1:length(f1ErrorTrack(:,1))
    relDisError1 = [relDisError1,norm(f1ErrorTrack(i,1:2))];
end
plot((1:length(relDisError1))*dt,relDisError1);
plot(((1:length(f1ErrorTrack(:,3)))*dt),f1ErrorTrack(:,3));
legend("Separation Distance Error","Bearing Angle Error")
subplot(2,1,2)
hold on
% plot(1:length(f2ErrorTrack(:,1)),f2ErrorTrack(:,1));
% plot(1:length(f2ErrorTrack(:,2)),f2ErrorTrack(:,2));
relDisError2 = [];
for i = 1:length(f2ErrorTrack(:,1))
    relDisError2 = [relDisError2,norm(f2ErrorTrack(i,1:2))];
end
plot((1:length(relDisError2))*dt,relDisError2);
plot((1:length(f2ErrorTrack(:,3)))*dt,f2ErrorTrack(:,3));
legend("Separation Distance Error","Bearing Angle Error")


%% Fig About the Velocity
figure
grid on
subplot(3,1,1)
hold on
plot((1:length(leaderStateTrack(:,1)))*dt,leaderStateTrack(:,4));
plot((1:length(leaderStateTrack(:,2)))*dt,leaderStateTrack(:,5));
legend("Linear Speed","Angular Speed")
subplot(3,1,2)
hold on
plot((1:length(fo1StateTrack(:,1)))*dt,fo1StateTrack(:,4));
plot((1:length(fo1StateTrack(:,2)))*dt,fo1StateTrack(:,5));
legend("Linear Speed","Angular Speed")
subplot(3,1,3)
hold on
plot((1:length(fo2StateTrack(:,1)))*dt,fo2StateTrack(:,4));
plot((1:length(fo2StateTrack(:,2)))*dt,fo2StateTrack(:,5));
legend("Linear Speed","Angular Speed")


