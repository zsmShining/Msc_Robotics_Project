clc;clear;
colorList = ["r","g","b"];
dt = 0.5;
colorList = ["r","g","b"];

track = [];
trackM = [];
formation = formation_lib.createFormation("line",[0,0,pi,1,pi/5],[0,0],4,pi/4);
list = formation.agent;

%% Virtual Target Tracking

dt = 0.1;
virList = ["g","g","g"];
colorList = ["r","g","b"];
vtall = 2;
vtsign = sign(vtall);
wtall = 0;
agt = agent(3,"follower",[2,1,0,0,0.1],[0,10]);
virAgt = list{3};
virAgt.angularSpeed = 0;
virAgtList = {virAgt};
agtList = {agt};
Tool_lib.plotScene(agtList,colorList);
Tool_lib.plotScene(virAgtList,virList);
% s
dt = 0.1;
cx = PIDcontroller([5,0.4,0.1],[0,6],dt);
cy = PIDcontroller([5.0,0.5,0.1],[-2,2],dt);
ctheta = PIDcontroller([0.9,0.5,0.01],[-2,2],dt);



sim_end = 100;
Error = agt.calPoseErrorinLocalFrame(virAgt);
cx = cx.oneStepControl(Error(1));
cy = cy.oneStepControl(Error(2));
ctheta = ctheta.oneStepControl(Error(3));

for t = 0:sim_end
Error = agt.calPoseErrorinLocalFrame(virAgt);
cx = cx.oneStepControl(Error(1));
cy = cy.oneStepControl(Error(2));
ctheta = ctheta.oneStepControl(Error(3));

vel = cx.output;
omega = cy.output+ctheta.output;
agt=agt.set("linearSpeed",vel);
agt=agt.set("angularSpeed",omega);

track = [track,agt.linearSpeed];
trackM = [trackM,agt.orientation];
agt=agt.nextStep(dt);
virAgt = virAgt.nextStep(dt);

virAgtList = {virAgt};
agtList = {agt};
Tool_lib.plotScene(agtList,colorList);
Tool_lib.plotScene(virAgtList,virList);
drawnow
if t~=sim_end
    clf();
else
    break;
end
end

figure
hold on
subplot(2,1,1)
plot(1:length(track),track);

subplot(2,1,2)
plot(1:length(trackM),trackM);






