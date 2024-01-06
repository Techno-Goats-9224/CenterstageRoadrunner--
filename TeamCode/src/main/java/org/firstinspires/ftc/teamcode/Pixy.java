package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

    public int readAvg(int queryAddress, int bytesToRead, int index, Telemetry telemetry){
        int retTotal = 0;
        for(int i = 1; i <= 25; i++){
            byte[] data = readShort(queryAddress, bytesToRead);
            retTotal += data[index];
            telemetry.addData("number of Signature", data[0]); // need this
            telemetry.addData("x position of largest block of that sig", data[1]);
            telemetry.addData("average signature 1 position", retTotal / i);
            telemetry.update();
        }
        return retTotal / 25;
    }
}