package edu.edina.Libraries.Robot;

public class BoundingBoxFailsafe {
    // forward arm/wrist conflict
    public static final double FORWARD_ARM_WRIST_CONFLICT_MIN = 0;
    public static final double FORWARD_ARM_WRIST_CONFLICT_MAX
            = 2 * Arm.INITIAL_DEGREES_BELOW_HORIZONTAL * Arm.TICKS_PER_DEGREE;
    public static final double HORIZONTAL_ARM_POSITION
            = Arm.INITIAL_DEGREES_BELOW_HORIZONTAL * Arm.TICKS_PER_DEGREE;

    // forward arm/slide conflict
    public static final double FORWARD_ARM_SLIDE_CONFLICT_LIMIT
            = FORWARD_ARM_WRIST_CONFLICT_MAX + 15 * Arm.TICKS_PER_DEGREE;

    public static final double WRIST_SAFE_POSITION = 0;
    public static final double SLIDE_TOL = 0.1;

    private final Wrist wrist;
    private final Arm arm;
    private final Lift lift;
    private final Slide slide;

    public BoundingBoxFailsafe(Wrist wrist, Arm arm, Lift lift, Slide slide) {
        this.wrist = wrist;
        this.arm = arm;
        this.lift = lift;
        this.slide = slide;
    }

    public void apply() {
        double armPos = arm.getCurrentPosition();
        double wristPos = wrist.getEstimatedPosition();
        double slidePos = slide.getPosition();

        if (FORWARD_ARM_WRIST_CONFLICT_MIN < armPos && armPos < FORWARD_ARM_WRIST_CONFLICT_MAX) {
            if (wristPos != WRIST_SAFE_POSITION) {
                double power = armPos < HORIZONTAL_ARM_POSITION ? -1 : 1;
                arm.overridePower(power);
                return;
            }
        }

        if (armPos < FORWARD_ARM_SLIDE_CONFLICT_LIMIT && Math.abs(slidePos) > SLIDE_TOL) {
           // slide.overridePower(-1);
            arm.overridePower(1);
            return;
        }


    }
}
