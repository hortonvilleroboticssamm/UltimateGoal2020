package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "BNO055", description = "IMU", xmlTag = "BNO055")
public class BNO055 extends I2cDeviceSynchDevice<I2cDeviceSynch> {


    private final I2cAddr ADDRESS_I2C_DEFUALT = new I2cAddr(12);

    public BNO055(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFUALT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public enum Register {
        FIRST(0),
        EUL_Pitch_MSB(0x1F),
        EUL_Roll_MSB(0x1D),
        EUL_Heading_MSB(0x1B),
        GYR_DATA_Z_MS(0x19),
        GYR_DATA_Y_MSB(0x17),
        GYR_DATA_X_MSB(0x15),
        MAG_DATA_Z_MSB(0x13),
        MAG_DATA_Y_MSB(0x11),
        MAG_DATA_X_MSB(0x9),
        ACC_DATA_Z_MSB(0xD),
        ACC_DATA_Y_MSB(0xB),
        ACC_DATA_X_MSB(0x9),
        RESOLUTION(20),
        LAST(RESOLUTION.i);

        public int i;

        Register(int i) {
            this.i = i;
        }
    }

    protected void setOptimalReadWindow(){
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(Register.FIRST.i, Register.LAST.i-Register.FIRST.i + 1, I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
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
        return "BNO055";
    }

    protected void writeShort(final Register reg, short value){
        deviceClient.write(reg.i, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg){
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.i, 2));
    }

}
