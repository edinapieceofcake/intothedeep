package edu.edina.Libraries.Robot;

public class MotionControlSettings {
    public MotionControlSettings(double ks, double kv, double ka,
                                 double velLimit, double maxPower,
                                 double posTolerance, double velTolerance,
                                 double pCoefficient) {

        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.velLimit = velLimit;
        this.maxPower = maxPower;
        this.accelLimit = maxPower / ka;
        this.posTolerance = posTolerance;
        this.velTolerance = velTolerance;
        this.pCoefficient = pCoefficient;
    }

    public final double ks, kv, ka;
    public final double velLimit, maxPower, accelLimit, posTolerance, velTolerance, pCoefficient;
}
