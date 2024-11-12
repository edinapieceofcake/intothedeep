package edu.edina.Libraries.Robot;

public class BoundingBoxFailsafe {
    private final Wrist wrist;
    private final Arm arm;
    private final Lift lift;
    private final Slide slide;

    public BoundingBoxFailsafe(Wrist wrist, Arm arm, Lift lift ,Slide slide){
        this.wrist = wrist;
        this.arm = arm;
        this.lift = lift;
        this.slide = slide;
    }

    public void apply(){

    }
}
