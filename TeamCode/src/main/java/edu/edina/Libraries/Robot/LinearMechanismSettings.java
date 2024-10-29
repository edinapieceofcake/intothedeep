package edu.edina.Libraries.Robot;

public class LinearMechanismSettings {
    // basic power settings
    public final double ks, kv;

    // used during calibration
    public final double accelCalibrationTarget;

    public LinearMechanismSettings(double ks, double kv,
                                   double accelCalibrationTarget) {

        this.ks = ks;
        this.kv = kv;
        this.accelCalibrationTarget = accelCalibrationTarget;
    }
}
