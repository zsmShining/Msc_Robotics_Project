clc;clear; close all;
%% Virtual Target Tracking

dt = 0.1;
virList = ["g","g","g"];
colorList = ["r","g","b"];
vtall = 2;
vtsign = sign(vtall);
wtall = 0;
agt = agent(1,"follower",[-2,0,0,vtall,wtall],[0,10]);
virAgt = agent(-1,"virtual",[-2,-1,0,1.5,0],[0,10]);
virAgtList = {virAgt};
agtList = {agt};
Tool_lib.plotScene(agtList,colorList);
Tool_lib.plotScene(virAgtList,virList);

dt = 0.1;
cx = PIDcontroller([4,0.4,0.4],[-0.5,2],dt);
cy = PIDcontroller([5.0,0.5,0.1],[-0.5,0.5],dt);
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



