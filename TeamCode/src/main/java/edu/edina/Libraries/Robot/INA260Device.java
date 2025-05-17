package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "INA260 Current/Power Monitor", xmlTag = "ina260")
public class INA260Device extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final int ADDRESS = 0x40;

    public INA260Device(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(new I2cAddr(ADDRESS));

        registerArmingStateCallback(false);

        this.deviceClient.engage();
    }

    public PowerReading getReading() {
        byte[] currBytes = deviceClient.read(0x01, 2);
        int amps = bytesToInt(currBytes[0], currBytes[1]);

        byte[] voltBytes = deviceClient.read(0x02, 2);
        int volts = bytesToInt(voltBytes[0], voltBytes[1]);
        return new PowerReading(amps * 0.00125, volts * 0.00125);
    }

    private static int bytesToInt(byte b0, byte b1) {
        int i0 = b0 & 0xff;
        int i1 = b1 & 0xff;
        int i = (i0 << 8) | i1;
        return i;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "INA260 Current/Power Monitor";
    }
}
