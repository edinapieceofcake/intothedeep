package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
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
public class NeoPixelDriverDevice extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final String LogTag = "NeoPixel";
    private static final int ADDRESS = 0x60;
    private static final byte NEOPIXEL_MODULE_BASE = 0xe, PIN = 0xf;
    private static final byte SPEED_400KHZ = 0x0, SPEED_800KHZ = 0x1;
    public static final short NUM_PIXELS = 18, NUM_BYTES = 3 * NUM_PIXELS;
    private static final int MAX_PIXEL_BYTE_BATCH = 24;
    private static final int WRITE_DELAY_MS = 25;
    private final BufferExchange bufExch;
    private AtomicInteger numWrites;

    public NeoPixelDriverDevice(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        RobotLog.ii(LogTag, "initialize NeoPixel driver @ 0x%x", ADDRESS);
        this.deviceClient.setI2cAddress(new I2cAddr(ADDRESS));

        registerArmingStateCallback(false);

        this.deviceClient.engage();

        bufExch = new BufferExchange();

        numWrites=new AtomicInteger();
    }

    public int getNumWrites() {
        return numWrites.get();
    }

    @Override
    protected boolean doInitialize() {
        initBoard();

        Timer t = new Timer();
        t.schedule(new FlushTask(), 0, 75);

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
        byte[] setPinCmd = new byte[]{
                PIN
        };

        write(NeoPixelSubModule.PIN, setPinCmd);
        RobotLog.ii(LogTag, "set NeoPixel pin to 0x%x", setPinCmd[0]);

        byte[] setSpeedCmd = new byte[]{
                SPEED_400KHZ
        };

        write(NeoPixelSubModule.SPEED, setSpeedCmd);
        RobotLog.ii(LogTag, "set NeoPixel speed to 0x%x", setSpeedCmd[0]);

        byte[] setNumBytesCmd = new byte[]{
                (byte) (NUM_BYTES >> 8),
                (byte) (NUM_BYTES)
        };

        write(NeoPixelSubModule.BUFLEN, setNumBytesCmd);
        RobotLog.ii(LogTag, "set NeoPixel buffer length to %d", NUM_BYTES);
    }

    private void flushColors() {
        byte[] pixArray = bufExch.viewBuffer();

        attemptToResynchWithDevice();

        if (pixArray != null) {
            int max = Math.min(pixArray.length, NUM_BYTES);

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
        // RobotLog.dd(LogTag, "writing %d bytes", cmd.length);

        sleep(WRITE_DELAY_MS);

        try {
            byte[] fullCmd = new byte[cmd.length + 2];
            fullCmd[0] = NEOPIXEL_MODULE_BASE;
            fullCmd[1] = sub.getVal();
            for (int i = 0; i < cmd.length; i++) {
                fullCmd[i + 2] = cmd[i];
            }

            deviceClient.write(fullCmd, I2cWaitControl.WRITTEN);
            numWrites.incrementAndGet();
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

    private void attemptToResynchWithDevice() {
//        byte[] empty = new byte[0];
//        for (int i = 0; i < 20; i++) {
//            sleep(5);
//            deviceClient.write(empty, I2cWaitControl.WRITTEN);
//            numWrites.incrementAndGet();
//        }
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
        @Override
        public void run() {
            flushColors();
        }
    }
}