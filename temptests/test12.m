clc;clear;close all;
fleet = formation_lib.createFormation("column",[0,0,pi/2,1,pi/20],[-20,20],4,pi/4);
colorList = ['r','g','b','k'];
Tool_lib.plotScene(fleet.agent,colorList);
obs1 = obstacle(5,[-10,10,0,0,0],2);
obs2 = obstacle(6,[-15,5,0,0,0],1);
leader = fleet.agent{1};
Kp = 1.0;
track = [];
follower = agent(2,"follower",[4,-1,0,0,0],[-10,5]);
dt = 0.1;
%KMatrix = [4,0.4,0.4;5.0,0.5,0.1;0.9,0.5,0.01];
% KMatrix = [4,0.4,0.1;5.0,0.5,0.1;0.9,0.5,0.01];
% KMatrix = [2,0.4,0.1;5.0,0.5,0.1;0.9,0.5,0.01];
KMatrix = [1,0.4,0.1;5.0,0.5,0.1;0.9,0.5,0.01];
% KMatrix = [4,1,0.5;5.0,0.5,0.1;0.9,0.5,0.01];
Boundary = [0,4;-1,1;-1,1];
Lamda = [1,2];
ctrlFormation = formation_controller(KMatrix,Boundary,Lamda,dt);
omega_track = [];
Etrck = [];

simStep = 400;

trackJ = [leader.position,follower.position];
for t = 1:simStep
    Tool_lib.plotScene(fleet.agent,colorList);
    Tool_lib.plotScene({follower},colorList);
    Tool_lib.plotScene({obs1,obs2},colorList);
    plot(trackJ(:,1),trackJ(:,2),"-");
    plot(trackJ(:,3),trackJ(:,4),"-");
%     plot(trackJ(:,5),trackJ(:,6),"-");
    drawnow
    if leader.checkIsAtGoal()
        break;
    end
    if t == simStep
        break;
    else
        clf();
    end
    leader = leader.nextStep(dt);
    virAgt = fleet.agent{2}; 
    Error = follower.calPoseErrorinLocalFrame(virAgt);
%    track = [track;Error'];
    ctrlFormation = ctrlFormation.oneStepControl(Error);
    res=ctrlFormation.output;
    Etrck = [Etrck;Error'];
    piddes_vel = res(1);
    des_vel = piddes_vel;
    des_omega = res(2);
    follower = follower.set("linearSpeed",piddes_vel);
    follower = follower.set("angularSpeed",des_omega);
    des_ang=Object_lib.convertBoundAng(des_omega*dt + follower.orientation);
    des_vel = des_vel * [cos(des_ang),sin(des_ang)];
    ndes_vel=follower.calAvoidCollision({obs2},des_vel);
    [vel,ang] = Object_lib.convertVelocity(ndes_vel);
    if vel == 0
        disp("meiyouzhehuishi")
        vec=(des_vel+ndes_vel)/2;
        [vel,ang] = Object_lib.convertVelocity(vec);
        disp("naahhh")
    end
    track = [track;vel];
    follower = follower.set("linearSpeed",vel);
    old = follower.orientation;
    diff = Object_lib.convertBoundAng(ang - old);
    omega = diff/dt;
    if abs(omega)>1
        omega = sign(omega)*1;
    end
    follower = follower.set("angularSpeed",omega);
    omega_track = [omega_track,omega];
 
    follower = follower.nextStep(dt);
    fleet = formation_lib.update(fleet,leader);
    trackJ = [trackJ;leader.position,follower.position];
    if t == simStep
        break;
    else
        clf();
    end
end



figure
hold on
plot(1:length(track(:,1)),track(:,1));
plot(1:length(omega_track),omega_track);
figure
hold on
plot(1:length(Etrck(:,1)),Etrck(:,1));
plot(1:length(Etrck(:,2)),Etrck(:,2));
plot(1:length(Etrck(:,3)),Etrck(:,3));