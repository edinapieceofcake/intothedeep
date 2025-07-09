package edu.edina.Libraries.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import edu.edina.Libraries.PurePursuit.Path;
import edu.edina.Libraries.PurePursuit.PurePursuitAction;

@Config
public class RobotDriver {
    private Drivetrain2 dt;
    private RobotState state;
    private ActionList actionList;
    private PurePursuitAction currentAction;

    public static double maxSpeed = 1, radius = 1;

    public RobotDriver(Drivetrain2 dt, RobotState state, ActionList actionList) {
        this.dt = dt;
        this.state = state;
        this.actionList = actionList;
    }

    public void addDrivePath(Vector2d[] vectors, double tgtSpeed) {
        if (currentAction != null) {
            currentAction.cancel();
            currentAction = null;
        }

        currentAction = new PurePursuitAction(new Path(vectors, tgtSpeed, maxSpeed, radius), dt, state);

        actionList.add(currentAction);
    }

    public void addDrivePath(Vector2d[] vectors, double tgtSpeed, double finalHeading) {
        if (currentAction != null) {
            currentAction.cancel();
            currentAction = null;
        }

        currentAction = new PurePursuitAction(new Path(vectors, tgtSpeed, maxSpeed, radius, finalHeading), dt, state);

        actionList.add(currentAction);
    }
}