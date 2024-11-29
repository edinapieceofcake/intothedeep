package edu.edina.Libraries.LinearMotion;

import android.annotation.SuppressLint;

public class LinearMechanismSettings {
    // info
    public final String name, units;

    // basic power settings
    public final double ks, kv, ka; // use CalibrateLinearMechanism op mode to help find these
    public final double nominalAccel, stopAccel0, stopAccel1;
    public final double stopTTol, stopXTol, stopVTol;
    public final double maxSpeed; // calculated
    public final double maxJerk; // maximum rate of change of the acceleration

    // used during calibration
    public final double accelCalibrationDist;

    public LinearMechanismSettings(double ks, double kv, double ka,
                                   double accelCalibrationDist) {
        this(null, null, ks, kv, ka, accelCalibrationDist);
    }

    public LinearMechanismSettings(String name, String units,
                                   double ks, double kv, double ka,
                                   double accelCalibrationDist) {
        this(name, units, ks, kv, ka, accelCalibrationDist, 1 / (2 * ka), 0.2,
                0.25,
                0.05 * mv(ks, kv),
                0.1 * Math.pow(mv(ks, kv), 3));
    }

    private LinearMechanismSettings(String name, String units,
                                    double ks, double kv, double ka,
                                    double accelCalibrationDist,
                                    double nominalAccel,
                                    double stopAccelDelta,
                                    double stopTTol,
                                    double stopXTol,
                                    double maxJerk) {
        this.name = name;
        this.units = units;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.accelCalibrationDist = accelCalibrationDist;
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

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("LinearMechanismSettings(%s, %s, ks=%.2e, kv=%.2e, ka=%.2e, " +
                        "d=%.2f, a=%.2e, stopAccel=(lo=%.2e, hi=%.2e), stopTol=(t=%.2f, x=%.2f, v=%.2e), " +
                        "maxJerk=%.2e, maxSpeed=%.2e)",
                name, units, ks, kv, ka,
                accelCalibrationDist, nominalAccel, stopAccel0, stopAccel1,
                stopTTol, stopXTol, stopVTol,
                maxJerk, maxSpeed
        );
    }
}