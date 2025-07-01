package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;

public class BackgroundSensorReader<T> {
    private final Timer timer;
    private final Supplier<T> readSensor;

    private T lastValue;

    public BackgroundSensorReader(Supplier<T> readSensor, int msReadDelay) {
        this.readSensor = readSensor;

        lastValue = readSensor.get();

        timer = new Timer();

        ReadTask task = new ReadTask();
        timer.schedule(task, msReadDelay, msReadDelay);
    }

    public synchronized T getValue() {
        return lastValue;
    }

    private synchronized void setValue(T value) {
        lastValue = value;
    }

    private class ReadTask extends TimerTask {
        @Override
        public void run() {
            T value = readSensor.get();
            setValue(value);
        }
    }
}
