function [Mindis] = findMinObsDis(agt,objList)
    obsList = Object_lib.getNeighbors("obstacle",agt,objList);
    Mindis = inf;
    if isempty(obsList)
        return;
    end
    for i = 1:length(obsList)
        obs = obsList{i};
        calDis = norm(agt.position - obs.position);
        if Mindis > calDis
            Mindis = calDis;
        end
    end
    
    
end