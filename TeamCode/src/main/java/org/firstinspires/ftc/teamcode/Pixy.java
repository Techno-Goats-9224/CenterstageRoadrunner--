package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "PixyCam", description = "v1 camera from PixyCam", xmlTag = "Pixy")
public class Pixy extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.valueOf("Charmed Labs");
    }
    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }
    @Override
    public String getDeviceName() {
        return "PixyCam v1";
    }
    public Pixy(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x01));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();;
    }

    public enum Register
    {
        LARGEST_BLOCK_ALL(0x50),
        SIG_1(0x51),
        SIG_2(0x52),
        SIG_3(0x53),
        SIG_4(0x54),
        SIG_5(0x55),
        SIG_6(0x56),
        SIG_7(0x57);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    protected byte[] readShort(int queryAddress, int bytesToRead)
    {
        return deviceClient.read(queryAddress, bytesToRead);
    }
    protected byte[] readShort(int signature)
    {
        int queryAddress = Integer.parseInt("0x5") + signature;
        if(signature == 1){
            queryAddress = 0x51;
        }
        if(signature == 2){
            queryAddress = 0x52;
        }
        if(signature == 3){
            queryAddress = 0x53;
        }
        if(signature == 4){
            queryAddress = 0x54;
        }
        if(signature == 5){
            queryAddress = 0x55;
        }
        if(signature == 6){
            queryAddress = 0x56;
        }
        if(signature == 7){
            queryAddress = 0x57;
        }
        return deviceClient.read(queryAddress, 6);
    }
}