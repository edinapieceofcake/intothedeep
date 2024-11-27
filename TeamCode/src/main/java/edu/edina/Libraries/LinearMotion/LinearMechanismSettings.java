package edu.edina.Libraries.LinearMotion;

public class LinearMechanismSettings {
    // basic power settings
    public final double ks, kv, ka, nominalAccel, stopAccel0, stopAccel1;
    public final double stopTTol, stopXTol, stopVTol;
    public final double maxSpeed, maxJerk;

    // used during calibration
    public final double accelCalibrationTarget;

    public LinearMechanismSettings(double ks, double kv, double ka,
                                   double accelCalibrationTarget) {
        this(ks, kv, ka, accelCalibrationTarget, 1 / (2 * ka), 0.2,
                0.25,
                0.25 * mv(ks, kv),
                0.1 * Math.pow(mv(ks, kv), 3));
    }

    private LinearMechanismSettings(double ks, double kv, double ka,
                                    double accelCalibrationTarget,
                                    double nominalAccel,
                                    double stopAccelDelta,
                                    double stopTTol,
                                    double stopXTol,
                                    double maxJerk) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.accelCalibrationTarget = accelCalibrationTarget;
        this.nominalAccel = nominalAccel;
        this.stopAccel0 = nominalAccel * (1 + stopAccelDelta);
        this.stopAccel1 = nominalAccel / (1 + stopAccelDelta);
        this.stopTTol = stopTTol;
        this.stopXTol = stopXTol;
        this.stopVTol = stopTTol * stopXTol;
        this.maxJerk = maxJerk;
        this.maxSpeed = mv(ks, kv);
    }

    private static double mv(double ks, double kv) {
        return (1 - ks) / kv;
    }
}