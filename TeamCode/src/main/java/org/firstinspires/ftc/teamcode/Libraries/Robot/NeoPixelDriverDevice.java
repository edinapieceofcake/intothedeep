package org.firstinspires.ftc.teamcode.Libraries.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "NeoPixel Driver", xmlTag = "neoPixelDriver")
public class NeoPixelDriverDevice extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final int ADDRESS = 0x60;
    private static final byte NEOPIXEL_MODULE_BASE = 0xe, PIN = 0xf;
    private static final short NUM_PIXELS = 32, NUM_BYTES = 3 * NUM_PIXELS;

    public NeoPixelDriverDevice(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(new I2cAddr(ADDRESS));

        registerArmingStateCallback(false);

        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        byte[] setPinCmd = new byte[]{
                PIN
        };

        write(NeoPixelSubModule.PIN, setPinCmd);

        byte[] setNumBytesCmd = new byte[]{
                (byte) (NUM_BYTES >> 8),
                (byte) (NUM_BYTES)
        };

        write(NeoPixelSubModule.BUFLEN, setNumBytesCmd);

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
        int max = Math.min(pixArray.length, NUM_BYTES);

        for (int start = 0; start < max; start += 24) {
            int stop = Math.min(start + 24, max);
            byte[] batch = new byte[stop - start + 2];
            batch[0] = (byte) (start >> 8);
            batch[1] = (byte) start;

            for (int i = 0; i + 2 < batch.length ; i++) {
                batch[i + 2] = pixArray[start + i];
            }

            write(NeoPixelSubModule.BUF, batch);
        }

        byte[] showCmd = new byte[0];

        write(NeoPixelSubModule.SHOW, showCmd);
    }

    private void write(NeoPixelSubModule sub, byte[] cmd) {
        byte[] fullCmd = new byte[cmd.length + 2];
        fullCmd[0] = NEOPIXEL_MODULE_BASE;
        fullCmd[1] = sub.getVal();
        for (int i = 0; i < cmd.length; i++) {
            fullCmd[i + 2] = cmd[i];
        }

        deviceClient.write(fullCmd);
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }
}