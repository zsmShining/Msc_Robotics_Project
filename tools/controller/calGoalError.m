function [Error]=calGoalError(agt)
% error in angle: must be buonded within [-pi,pi]
    if agt.type == "agent" && agt.role == "leader"
        goalPos = agt.goal;
        curPos = agt.position;
        diff = goalPos - curPos;
        theta_goal = atan2(diff(2),diff(1));
        theta_actual = agt.orientation;
        Error = theta_goal - theta_actual;
        Error = atan2(sin(Error),cos(Error));
    else
        error("wrong type of agent")
    end

end