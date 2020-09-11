clc;clear;close all;
Ragt1 = agent(1,"leader",[0,0,pi/2,1,0],[0,10]);
agt2 = agent(2,"follower",[0,0,pi/2,1,0],[0,10]);
virAgt = agent(-2,"virtual",[2,2,pi/3,1,0],[0,10]);

agtList = {agt2,virAgt};
colorList = ['r','g','b'];
Tool_lib.plotScene(agtList,colorList);


for i = 1:length(agtList)
    agt = agtList{i};
    agtList{i} = agt.nextStep(1);
end
Tool_lib.plotScene(agtList,colorList);




thetaF = agt2.orientation;
rot = [cos(thetaF),sin(thetaF),0; ...
       -sin(thetaF),cos(thetaF),0; ...
       0,         0,         1];
deltaR = virAgt.get("state")-agt2.get("state");
deltaR = deltaR(1:3);

error = rot * deltaR'

deltP = 0.5;deltT = 0.5;
delta = [deltP,deltP,deltT];

% angular Speed
ey_max = error(2);
ex = error(1);
ey = error(2);
if abs(ey)>= deltP || ex<(-deltP*(abs(ey)<deltP))
    % TT
else
end
    
   







