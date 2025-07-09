package edu.edina.Libraries.PurePursuit;

public class MotionCommand {
    public final double axial, lateral, yaw;

    public MotionCommand(double axial, double lateral, double yaw) {
        this.axial = axial;
        this.lateral = lateral;
        this.yaw = yaw;
    }

    public MotionCommand rescale() {
        double x = Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw);
        if (x <= 1)
            return this;

        return new MotionCommand(axial / x, lateral / x, yaw / x);
    }
}
