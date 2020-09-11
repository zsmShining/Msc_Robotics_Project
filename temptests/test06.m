clc;clear;
colorList = ["r","g","b"];
dt = 0.5;
colorList = ["r","g","b"];

    
formation = formation_lib.createFormation("line",[0,0,pi,1,pi/5],[0,0],4,pi/4);
list = formation.agent;
dt = 0.1;
virList = ["g","g","g"];
colorList = ["r","g","b"];
vtall = 2;
vtsign = sign(vtall);
wtall = 0;
agt = agent(3,"follower",[15,3,pi,4,0.7],[0,10]);
virAgt = list{3};
virAgtList = {virAgt};
agtList = {agt};
Tool_lib.plotScene(agtList,colorList);
Tool_lib.plotScene(virAgtList,virList);




