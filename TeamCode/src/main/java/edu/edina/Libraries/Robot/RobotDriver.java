package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;

import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.PurePursuit.PurePursuitAction;

@Config
public class RobotDriver {
    private Drivetrain dt;
    private RobotState state;
    private ActionList actionList;
    private PurePursuitAction currentAction;

    public static double maxSpeed = 24, radius = 5;

    public RobotDriver(Drivetrain dt, RobotState state, ActionList actionList) {
        this.dt = dt;
        this.state = state;
        this.actionList = actionList;
    }

    public Action addDrivePath(Path path) {
        if (currentAction != null) {
            currentAction.cancel();
            currentAction = null;
        }

        currentAction = new PurePursuitAction(path, dt, state);

        actionList.add(currentAction);

        return currentAction;
    }
}