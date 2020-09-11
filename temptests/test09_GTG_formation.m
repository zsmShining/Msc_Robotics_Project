clc;clear;close all;
fleet = formation_lib.createFormation("column",[0,0,pi/2],[-20,20],4,pi/4);
colorList = ['r','g','b','k'];
Tool_lib.plotScene(fleet.agent,colorList);   
leader = fleet.agent{1};
Kp = 1.0;
track = [];
follower = agent(2,"follower",[4,-1,0,pi/2,0],[-10,5]);
dt = 0.1;
KMatrix = [4,0.4,0.4;5.0,0.5,0.1;0.9,0.5,0.01];Boundary = [0,4;-1,1;-1,1];
Lamda = [1,1];
ctrlFormation = formation_controller(KMatrix,Boundary,Lamda,dt);

simStep = 200;
trackJ = [leader.position,fleet.agent{2}.position,fleet.agent{3}.position];
for t = 1:simStep
    Tool_lib.plotScene(fleet.agent,colorList);
    Tool_lib.plotScene({follower},colorList);
    plot(trackJ(:,1),trackJ(:,2),"-");
    plot(trackJ(:,3),trackJ(:,4),"-");
    plot(trackJ(:,5),trackJ(:,6),"-");
    drawnow
    if leader.checkIsAtGoal()
        break;
    end
    if t == simStep
        break;
    else
        clf();
    end
    
    goal=leader.calGoalVector();
    theta_actual = leader.orientation;
    [~,theta_desired] = Object_lib.convertVelocity(goal);
    thetaE = theta_desired - theta_actual;
    thetaE = atan2(sin(thetaE),cos(thetaE));
    
    trackJ = [trackJ;leader.position,fleet.agent{2}.position,fleet.agent{3}.position]; 
    w_c = Kp * thetaE;
    leader = leader.set("angularSpeed",w_c);
    leader = leader.nextStep(dt);
    virAgt = fleet.agent{2}; 
    Error = follower.calPoseErrorinLocalFrame(virAgt);
    track = [track,Error(2)];
    ctrlFormation = ctrlFormation.oneStepControl(Error);
    res=ctrlFormation.output;
    vel = res(1);
    omega = res(2);
    follower = follower.set("linearSpeed",vel);
    follower = follower.set("angularSpeed",omega);
    follower = follower.nextStep(dt);
    fleet = formation_lib.update(fleet,leader);
end



figure
plot(1:length(track),track)

