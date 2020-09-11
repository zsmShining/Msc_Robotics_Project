clc;clear;close all;
agt1 = agent(1,"leader",[0,0,pi/2,0,0],[10,10]);


list = {agt1};%, agt2};
colorList = ["r","b","g"];
newList = list;
dt = 0.05;
xtheta = PIDcontroller([1,1,0.1],[0,2],dt);
ctheta = PIDcontroller([2,0.5,0.01],[-2,2],dt);
sim_end = 150;
track = [];
nt = [];
for t = 1:sim_end
Tool_lib.plotScene(list,colorList)
scatter(agt1.goal(1),agt1.goal(2))
for i= 1:length(list)
    obj = list{i};
    if obj.type == "agent"
            if obj.id ==1
            track = [track,obj.angularSpeed];
            end
            velocity=obj.calMovetoGoal();
            [linearSpeed,theta] = Object_lib.convertVelocity(velocity); 
            drror = norm(obj.goal-obj.position);            
            xtheta = xtheta.oneStepControl(drror);           
            linearSpeed = xtheta.output;
            obj = obj.set("linearSpeed",linearSpeed);
            error = theta - obj.orientation;
            ctheta = ctheta.oneStepControl(error);
            omega = ctheta.output;
            obj = obj.set("angularSpeed",omega);
            obj = obj.nextStep(dt);      
    else
        obj = obj.nextStep(dt);
    end
    newList{i} = obj;
end
drawnow
list = newList;

if t ~= sim_end
    clf();
end
end
figure
plot(1:length(track),track);