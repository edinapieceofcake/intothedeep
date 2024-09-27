package edu.edina.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;

@Autonomous
public class OldNeoPixelTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NeoPixel neoPixel = hardwareMap.get(NeoPixel.class, "neopixel_driver");

        waitForStart();

        neoPixel.showColors();

        while (opModeIsActive()) {
            neoPixel.showColors();
        }
    }

    public static class NeoPixel extends I2cDeviceSynchDevice<I2cDeviceSynch> {
        private static final int ADDRESS = 0x60;
        private static final byte MODULE_BASE = 0xe, PIN = 0xf;
        private static final short NUM_PIXELS = 2, NUM_BYTES = 3 * NUM_PIXELS;

        private enum NeoPixelSubModule {
//            NEOPIXEL_STATUS(0x0),
            PIN(0x1),
//            NEOPIXEL_SPEED(0x2),
            BUFLEN(0x3),
            BUF(0x4),
            SHOW(0x5);

            public int val;

            NeoPixelSubModule(int val) {
                this.val = val;
            }

            public byte getVal() {
                return (byte) val;
            }
        }

        public NeoPixel(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
            super(deviceClient, deviceClientIsOwned);

            this.deviceClient.setI2cAddress(new I2cAddr(ADDRESS));

            registerArmingStateCallback(false);

            this.deviceClient.engage();
        }

        @Override
        protected boolean doInitialize() {
            byte[] setPinCmd = new byte[]{
                    MODULE_BASE,
                    NeoPixelSubModule.PIN.getVal(),
                    PIN
            };
            deviceClient.write(setPinCmd);
            deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);

            byte[] setNumBytesCmd = new byte[]{
                    MODULE_BASE,
                    NeoPixelSubModule.BUFLEN.getVal(),
                    (byte) (NUM_BYTES >> 8),
                    (byte) (NUM_BYTES)
            };
            deviceClient.write(setNumBytesCmd);
            deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);

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

        public void showColors() {
            // write maximum 30 bytes at a time
            byte[] setBufCmd = new byte[NUM_BYTES + 4];
            setBufCmd[0] = MODULE_BASE;
            setBufCmd[1] = NeoPixelSubModule.BUF.getVal();
            setBufCmd[2] = 0;
            setBufCmd[3] = 0;
            for (int i = 0; i < NUM_BYTES; i++)
                setBufCmd[i + 4] = (byte) 0xff;
            deviceClient.write(setBufCmd);

            byte[] showCmd = new byte[]{
                    MODULE_BASE,
                    NeoPixelSubModule.SHOW.getVal()
            };
            deviceClient.write(showCmd);
            deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
        }
    }
}
