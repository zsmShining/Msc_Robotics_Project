clc;clear;
formation = formation_lib.createFormation("line",[0,0,pi],[0,10],4,pi/4);
colorList = ['r','g','b','k'];

leader = formation.agent{1};
follower1 = agent(2,"follower",[4,1,pi],[0,0]);
follower2 = agent(3,"follower",[8,1,pi],[0,0]);
 
obs1 = obstacle(4,[-10,0,0,0,0]);
objList = {leader,follower1,follower2,obs1};
dt = 0.1;
objList = {leader};
control_goalHeading=PIDcontroller([1,1,0],[-0.5,0.5],dt);
control_goalDis= PIDcontroller([2,1,1],[-4,4],dt);
for i = 1:100
    scatter(leader.goal(1),leader.goal(2));
Tool_lib.plotScene(objList,colorList);    
drawnow
lgv = leader.calMovetoGoal();
[desLinearSpeed,desHeading] = Object_lib.convertVelocity(lgv);
errorHeading = desHeading-leader.orientation;
control_goalHeading = control_goalHeading.oneStepControl(errorHeading);
w_c = control_goalHeading.output;
diff = leader.position-leader.goal;
errorDis = -norm(diff)*cos(atan2(diff(2),diff(1))-leader.orientation);
control_goalDis = control_goalDis.oneStepControl(errorDis);
v_c = control_goalDis.output;
leader = leader.set("angularSpeed",w_c);
leader = leader.set("linearSpeed",v_c);
leader = leader.nextStep(dt);
objList = {leader};
if i == 100
    break;
end
clf()
end
