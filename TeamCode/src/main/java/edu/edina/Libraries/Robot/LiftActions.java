package edu.edina.Libraries.Robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;

import edu.edina.Libraries.Actions.ContinuousAction;
import edu.edina.Libraries.LinearMotion.LinearMotionController;
import edu.edina.Libraries.LinearMotion.SpringForce;
import edu.edina.Libraries.LinearMotion.VerticalExtensionMechanism;

public class LiftActions {
    public static double k = -3.88;
    public static double g = -9.70;
    public static double pMult=1;

    private SpringForce springForce;
    private VerticalExtensionMechanism vMech;

    public LiftActions(RobotHardware hw) {
        springForce = new SpringForce(k, g);
        vMech = new VerticalExtensionMechanism(hw);
    }

    public Action setTarget(double heightInches) {
        LinearMotionController lmc = new LinearMotionController(vMech, springForce);
        lmc.setTarget(heightInches);
        return new ContinuousAction(lmc::run);
    }

    public Action holdPos(double heightInches) {
        return new ContinuousAction(() -> {
            DualNum<Time> current = vMech.getPositionAndVelocity(false);
            double a = springForce.getAcceleration(current);
            double feedForward = a * vMech.getSettings().ka;
            double pTerm = pMult * (heightInches - current.get(0));
            double power = feedForward + pTerm;
            vMech.setPower(power);
            return true;
        });
    }

    public double getCurrent() {
        return vMech.getCurrent();
    }

    public VerticalExtensionMechanism getVMech() {
        return vMech;
    }
}