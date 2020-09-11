clc;clear;close all;
fleet = formation_lib.createFormation("column",[0,0,pi],[-10,10],4,pi/4);
colorList = ['r','g','b','k'];
Tool_lib.plotScene(fleet.agent,colorList);   
leader = fleet.agent{1};
Kp = 0.6;
track = [];
dt=0.1;
simStep = 100;
weightGoal=weightCalculator();
trackJ = [leader.position,fleet.agent{2}.position,fleet.agent{3}.position];
for t = 1:simStep
    Tool_lib.plotScene(fleet.agent,colorList);
    plot(trackJ(:,1),trackJ(:,2),"-");
    plot(trackJ(:,3),trackJ(:,4),"-");
    plot(trackJ(:,5),trackJ(:,6),"-");
    drawnow
    if t == simStep
        break;
    end
    clf()
    goal=leader.calGoalVector();
    theta_actual = leader.orientation;
    [~,theta_desired] = Object_lib.convertVelocity(goal);
    thetaE = theta_desired - theta_actual;
    thetaE = atan2(sin(thetaE),cos(thetaE));
    track = [track,fleet.agent{3}.linearSpeed];
    trackJ = [trackJ;leader.position,fleet.agent{2}.position,fleet.agent{3}.position]; 
    w_c = Kp * thetaE;
    
    leader = leader.set("angularSpeed",w_c);
    v_c = 2*weightGoal.forGoal(leader,dt);
    leader = leader.set("linearSpeed",v_c);
    leader = leader.nextStep(dt);
    
    virAgt2 = fleet.agent{2};
    
    fleet = formation_lib.update(fleet,leader);

end



figure
plot(1:length(track),track)

