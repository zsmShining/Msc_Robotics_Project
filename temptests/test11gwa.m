%% Combine goal with collision avoidance
clc;clear;close all;
warning on
leader = agent(1,"leader",[10,-10,pi/2],[-10,10]);
obs1 = obstacle(5,[0,0,0,0,0],3);
obs2 = obstacle(5,[0,10,0,0,0],2);
obs3 = obstacle(5,[-15,8,0,1,0],1);

colorList = ['r','g','b','k'];
tauT = 1/Object_lib.agentMaxLinearSpeed;
Kp = 1.0;
track = [];
follower = agent(2,"follower",[4,-1,0,0,0],[-10,5]);
dt = 0.1;
KMatrix = [4,0.4,0.4;5.0,0.5,0.1;0.9,0.5,0.01];Boundary = [0,4;-1,1;-1,1];
Lamda = [1,1];
ctrlFormation = formation_controller(KMatrix,Boundary,Lamda,dt);
track = [];
ntrack = [];
simStep = 500;
trackJ = [leader.position];
for t = 1:simStep
    Tool_lib.plotScene({leader},colorList);
Tool_lib.plotScene({obs1,obs2,obs3},colorList);
    plot(trackJ(:,1),trackJ(:,2),"-");
    drawnow
 
    goal=leader.calGoalVector();
    theta_actual = leader.orientation;
    [~,theta_desired] = Object_lib.convertVelocity(goal);
    thetaE = theta_desired - theta_actual;
    thetaE = atan2(sin(thetaE),cos(thetaE));
    w_c = Kp * thetaE;
    
    
    if leader.checkIsAtGoal()
        break;
    end
    if t == simStep
        break;
    else
        clf();
    end
    disp("++++")
    disp(leader.orientation+w_c*dt);
    ang = atan2(sin(leader.orientation+w_c*dt),cos(leader.orientation+w_c*dt));
    vector = leader.linearSpeed * [cos(ang),sin(ang)];
    velocity = leader.calAvoidCollision({obs1,obs2,obs3},vector);
    [vel,ang] = Object_lib.convertVelocity(velocity);
%     leader = leader.set("angularSpeed",w_c);   
   % leader = leader.set("linearSpeed",vel);
    old = leader.orientation;
    diff = ang - old;
    diff = atan2(sin(diff),cos(diff));
    leader = leader.set("angularSpeed",diff/dt);
%     leader = leader.set("angularSpeed",(dif)/tauT);
%     disp("ang")
%     disp(diff/dt);
    track = [track,diff/dt];
    ntrack = [ntrack,leader.angularSpeed];
%     disp(dt/tauT);
%     disp(w_c)
    leader = leader.nextStep(dt);
    disp(leader.orientation==ang);
    obs3 = obs3.nextStep(dt);
%     disp("++++")
end
figure
hold on
plot((1:length(track))*dt,track,'k');
plot((1:length(ntrack))*dt,ntrack,'m');
plot((1:length(track))*dt,Object_lib.agentMaxAngularSpeed*ones(length(track)),"r");
plot((1:length(track))*dt,-Object_lib.agentMaxAngularSpeed*ones(length(track)),"r");



