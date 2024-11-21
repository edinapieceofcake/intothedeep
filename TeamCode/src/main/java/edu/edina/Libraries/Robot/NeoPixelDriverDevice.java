package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Timer;
import java.util.TimerTask;

@I2cDeviceType
@DeviceProperties(name = "NeoPixel Driver", xmlTag = "neoPixelDriver")
public class NeoPixelDriverDevice extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final String LogTag = "NeoPixel";
    private static final int ADDRESS = 0x60;
    private static final byte NEOPIXEL_MODULE_BASE = 0xe, PIN = 0xf;
    public static final short NUM_PIXELS = 18, NUM_BYTES = 3 * NUM_PIXELS;
    private static final int MAX_PIXEL_BYTE_BATCH = 24;
    private static final int WRITE_DELAY_MS = 50;
    private final BufferExchange bufExch;

    public NeoPixelDriverDevice(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        RobotLog.ii(LogTag, "initialize NeoPixel driver @ 0x%x", ADDRESS);
        this.deviceClient.setI2cAddress(new I2cAddr(ADDRESS));

        registerArmingStateCallback(false);

        this.deviceClient.engage();

        bufExch = new BufferExchange();
    }

    @Override
    protected boolean doInitialize() {
        byte[] setPinCmd = new byte[]{
                PIN
        };

        write(NeoPixelSubModule.PIN, setPinCmd);
        RobotLog.ii(LogTag, "set NeoPixel pin to 0x%x", PIN);

        byte[] setNumBytesCmd = new byte[]{
                (byte) (NUM_BYTES >> 8),
                (byte) (NUM_BYTES)
        };

        write(NeoPixelSubModule.BUFLEN, setNumBytesCmd);
        RobotLog.ii(LogTag, "set NeoPixel buffer length to %d", NUM_BYTES);

        Timer t = new Timer();
        t.schedule(new FlushTask(), 0, 1);

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

    private void flushColors() {
        byte[] pixArray = bufExch.viewBuffer();
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

        byte[] showCmd = new byte[0];
        write(NeoPixelSubModule.SHOW, showCmd);
    }

    private void write(NeoPixelSubModule sub, byte[] cmd) {
        // RobotLog.dd(LogTag, "writing %d bytes", cmd.length);

        try {
            Thread.sleep(WRITE_DELAY_MS);
        } catch (InterruptedException x) {
            //
        }

        try {
            byte[] fullCmd = new byte[cmd.length + 2];
            fullCmd[0] = NEOPIXEL_MODULE_BASE;
            fullCmd[1] = sub.getVal();
            for (int i = 0; i < cmd.length; i++) {
                fullCmd[i + 2] = cmd[i];
            }

            deviceClient.write(fullCmd, I2cWaitControl.ATOMIC);
        } catch (RuntimeException x) {
            RobotLog.ee(LogTag, "could not write to NeoPixel driver %s", x);
            throw x;
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
        @Override
        public void run() {
            RobotLog.ii(LogTag, "flushing");
            flushColors();
        }
    }
}