package edu.edina.Libraries.Robot;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicInteger;

@I2cDeviceType
@DeviceProperties(name = "NeoPixel Driver", xmlTag = "neoPixelDriver")
public class NeoPixelDriverDevice extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, NeoPixelDriverDevice.Params> {
    public static class Params implements Cloneable {
        public I2cAddr i2cAddr = new I2cAddr(ADDRESS_DEFAULT);
        public int numPixels;

        public int getNumBytes() {
            return 3 * numPixels;
        }

        @NonNull
        @Override
        public Params clone() {
            try {
                return (Params) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new AssertionError();
            }
        }
    }

    private static final String LogTag = "NeoPixel";
    private static final int ADDRESS_DEFAULT = 0x60;
    private static final byte NEOPIXEL_MODULE_BASE = 0xe, PIN = 0xf;
    private static final byte SPEED_400KHZ = 0x0, SPEED_800KHZ = 0x1;
    private static final int MAX_PIXEL_BYTE_BATCH = 24;
    private static final int WRITE_DELAY_MS = 5;

    private final BufferExchange bufExch;
    private Timer timer;
    private FlushTask flushTask;

    public NeoPixelDriverDevice(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned, new Params());

        registerArmingStateCallback(false);

        this.deviceClient.engage();

        bufExch = new BufferExchange();

        timer = new Timer();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        RobotLog.ii(LogTag, "resetDeviceConfigurationForOpMode");
        showColors(new byte[parameters.getNumBytes()]);

        if (flushTask != null) {
            flushTask.cancel();
            flushTask = null;
        }

        super.resetDeviceConfigurationForOpMode();
    }

    @Override
    protected boolean internalInitialize(@NonNull Params parameters) {
        RobotLog.ii(LogTag, "internalInitialize");
        this.parameters = parameters;

        if (parameters != defaultParameters) {
            initBoard();

            RobotLog.ii(LogTag, "scheduling new timer task");

            flushTask = new FlushTask();
            timer.schedule(flushTask, 10, 75);
        }

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "NeoPixel Driver";
    }

    //pixel order (green, red, blue)
    public void showColors(byte[] pixArray) {
        bufExch.updateBuffer(pixArray);
    }

    private void initBoard() {
        RobotLog.ii(LogTag, "address set to 0x%x", parameters.i2cAddr.get7Bit());
        this.deviceClient.setI2cAddress(parameters.i2cAddr);

        byte[] setPinCmd = new byte[]{
                PIN
        };

        write(NeoPixelSubModule.PIN, setPinCmd);
        RobotLog.ii(LogTag, "set NeoPixel pin to 0x%x", setPinCmd[0]);

        byte[] setSpeedCmd = new byte[]{
                SPEED_800KHZ
        };

        write(NeoPixelSubModule.SPEED, setSpeedCmd);
        RobotLog.ii(LogTag, "set NeoPixel speed to 0x%x", setSpeedCmd[0]);

        int numBytes = parameters.getNumBytes();
        byte[] setNumBytesCmd = new byte[]{
                (byte) (numBytes >> 8),
                (byte) (numBytes)
        };

        write(NeoPixelSubModule.BUFLEN, setNumBytesCmd);
        RobotLog.ii(LogTag, "set NeoPixel buffer length to %d", numBytes);
    }

    private void flushColors(int numBytes) {
        byte[] pixArray = bufExch.viewBuffer();

        if (pixArray != null) {
            int max = Math.min(pixArray.length, numBytes);

            for (int start = 0; start < max; start += MAX_PIXEL_BYTE_BATCH) {
                int stop = Math.min(start + MAX_PIXEL_BYTE_BATCH, max);
                byte[] batch = new byte[stop - start + 2];
                batch[0] = (byte) (start >> 8);
                batch[1] = (byte) start;

                for (int i = 0; i + 2 < batch.length; i++) {
                    batch[i + 2] = pixArray[start + i];
                }

                write(NeoPixelSubModule.BUF, batch);
            }
        }

        byte[] empty = new byte[0];
        write(NeoPixelSubModule.SHOW, empty);
    }

    private void write(NeoPixelSubModule sub, byte[] cmd) {
        sleep(WRITE_DELAY_MS);

        try {
            byte[] fullCmd = new byte[cmd.length + 2];
            fullCmd[0] = NEOPIXEL_MODULE_BASE;
            fullCmd[1] = sub.getVal();
            for (int i = 0; i < cmd.length; i++) {
                fullCmd[i + 2] = cmd[i];
            }

            deviceClient.write(fullCmd, I2cWaitControl.WRITTEN);
        } catch (RuntimeException x) {
            RobotLog.ee(LogTag, "could not write to NeoPixel driver %s", x);
            throw x;
        }
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException x) {
            // ignore
        }
    }

    private enum NeoPixelSubModule {
        PIN(0x1),
        SPEED(0x2),
        BUFLEN(0x3),
        BUF(0x4),
        SHOW(0x5);

        public final int val;

        NeoPixelSubModule(int val) {
            this.val = val;
        }

        public byte getVal() {
            return (byte) val;
        }
    }

    private static class BufferExchange {
        private byte[] buffer;

        public synchronized void updateBuffer(byte[] buffer) {
            this.buffer = buffer.clone();
        }

        public synchronized byte[] viewBuffer() {
            return this.buffer;
        }
    }

    private class FlushTask extends TimerTask {
        private final int numBytes;

        public FlushTask() {
            numBytes = parameters.getNumBytes();
        }

        @Override
        public void run() {
            flushColors(numBytes);
        }
    }
}