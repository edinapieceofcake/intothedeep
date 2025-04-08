package edu.edina.Libraries.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Timer;
import java.util.concurrent.atomic.AtomicInteger;

@I2cDeviceType
@DeviceProperties(name = "INA219 Current/Power Monitor", xmlTag = "ina219")
public class INA219Device extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final int ADDRESS = 0x40;

    public INA219Device(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(new I2cAddr(ADDRESS));

        registerArmingStateCallback(false);

        this.deviceClient.engage();
    }

    public byte[] getReading() {
        return deviceClient.read(0x01, 2);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "INA219 Current/Power Monitor";
    }
}
