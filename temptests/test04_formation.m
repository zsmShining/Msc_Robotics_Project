clc;clear;
colorList = ["r","g","b"];
vshape = formation_lib.createFormation("vshape",[0,0,pi/2,1.5,0.1],[0,10],4,pi/4);
follower1 = agent(2,"follower",[2,-4,pi/2,3,0],[0,0]);
follower2 = agent(3,"follower",[-2,-4,pi/2,2,0],[0,0]);
followerList = {follower1,follower2};
dt = 0.1;

cx1 = PIDcontroller([4,0.4,0.4],[-0.5,2],dt);
cy1 = PIDcontroller([5.0,0.5,0.1],[-0.5,2],dt);
ctheta1 = PIDcontroller([0.9,0.5,0.01],[-2,2],dt);
PIDList = [cx1,cy1,ctheta1];
cx2 = PIDcontroller([4,0.4,0.4],[-0.5,2],dt);
cy2 = PIDcontroller([5.0,0.5,0.1],[-0.5,2],dt);
ctheta2 = PIDcontroller([0.9,0.5,0.01],[-2,2],dt);
PIDList = [PIDList;cx2,cy2,ctheta2];

for t = 1:1
for i = 1:length(vshape.agent)
    Tool_lib.plotScene(vshape.agent,colorList);
    Tool_lib.plotScene(followerList,colorList);
    drawnow
    if t ~= 2
    clf()
else
    break;
end
    if i == 1
        agt = vshape.agent{1};
        Updatedleader = agt.nextStep(dt);
    else
%         if i == 3
%             continue
%         end
        cx = PIDList(i-1,1);cy = PIDList(i-1,2);ctheta = PIDList(i-1,3);
        agt = followerList{i-1};
        virAgt = vshape.agent{i};
        Error = agt.calPoseErrorinLocalFrame(virAgt);
        cx = cx.oneStepControl(Error(1));
        cy = cy.oneStepControl(Error(2));
        ctheta = ctheta.oneStepControl(Error(3));
        vel = cx.output;
        omega = cy.output+ctheta.output;
        agt=agt.set("linearSpeed",vel);
        agt=agt.set("angularSpeed",omega);
        agt=agt.nextStep(dt);
        followerList{i-1} = agt;
        PIDList(i-1,1) = cx;PIDList(i-1,2)=cy;PIDList(i-1,3)= ctheta;
    end
end
vshape = formation_lib.update(vshape,Updatedleader);

end




