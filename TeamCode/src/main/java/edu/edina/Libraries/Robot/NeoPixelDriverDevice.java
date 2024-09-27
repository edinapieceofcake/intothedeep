package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import edu.edina.OpModes.Autonomous.OldNeoPixelTest;

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
                NEOPIXEL_MODULE_BASE,
                NeoPixelSubModule.PIN.getVal(),
                PIN
        };

        write(setPinCmd);

        byte[] setNumBytesCmd = new byte[]{
                NEOPIXEL_MODULE_BASE,
                NeoPixelSubModule.BUFLEN.getVal(),
                (byte) (NUM_BYTES >> 8),
                (byte) (NUM_BYTES)
        };

        write(setNumBytesCmd);

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

    private void write(byte[] cmd) {
        deviceClient.write(cmd);
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }

    public void showColors() {
        // write maximum 30 bytes at a time
        byte[] setBufCmd = new byte[NUM_BYTES + 4];
        setBufCmd[0] = NEOPIXEL_MODULE_BASE;
        setBufCmd[1] = NeoPixelSubModule.BUF.getVal();
        setBufCmd[2] = 0;
        setBufCmd[3] = 25*3;
        for (int i = 0; i < NUM_BYTES; i++)
            setBufCmd[i + 4] = (byte) 0x10;
        deviceClient.write(setBufCmd);

        byte[] showCmd = new byte[]{
                NEOPIXEL_MODULE_BASE,
                NeoPixelSubModule.SHOW.getVal()
        };
        deviceClient.write(showCmd);
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }
}