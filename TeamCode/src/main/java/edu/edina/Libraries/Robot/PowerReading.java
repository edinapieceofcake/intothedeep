package edu.edina.Libraries.Robot;

public class PowerReading {
    private double current, voltage;

    public PowerReading(double current, double voltage) {
        this.current = current;
        this.voltage = voltage;
    }

    public double getCurrent() {
        return current;
    }

    public double getVoltage() {
        return voltage;
    }
}
