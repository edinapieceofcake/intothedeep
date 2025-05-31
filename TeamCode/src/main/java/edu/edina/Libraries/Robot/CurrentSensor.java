package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.edina.Tests.DataFile;

public class CurrentSensor {
    private INA260Device dev;
    private ElapsedTime t;
    private DataFile dataFile;

    public CurrentSensor(HardwareMap hw) {
        dev = hw.get(INA260Device.class, "sensor_ina260");
    }

    public void checkForInit() {
        if (dataFile == null) {
            PowerReading r = dev.getReading();
            if (Math.abs(r.getCurrent()) > 1e-4) {
                dataFile = new DataFile("current-monitor.csv");
                dataFile.println("Time,Current,Voltage");

                t = new ElapsedTime();
            }
        }
    }

    public void update() {
        if (dataFile != null) {
            PowerReading r = dev.getReading();
            dataFile.println(String.format("%f,%f,%f", t.seconds(), r.getCurrent(), r.getVoltage()));
        }
    }
}
