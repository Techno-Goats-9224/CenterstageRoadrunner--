package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */
@TeleOp
public class PixyMess extends OpMode {
    /*public enum Register
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
    }*/
    ElapsedTime runtime = new ElapsedTime();
    private Pixy pixy;

    int starterStack = 0;
    @Override
    public void init() {
        pixy = hardwareMap.get(Pixy.class, "pixy");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // run until the end of the match (driver presses STOP)
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        //for(Pixy.Register reg : Pixy.Register.values()){
        /*for(int reg = 80; reg < 88; reg++){
            byte[] data = pixy.readShort(reg, 8);
            for(int j = 0; j < 8; j++) {
                telemetry.addData(reg + " " + j, data[j]);
            }
        }*/
        byte[] pixyBytes = pixy.readShort(0x51, 5);
        runtime.reset();
        telemetry.addData("number of Signature 1", pixyBytes[0]);
        telemetry.addData("x position of largest block", pixyBytes[1]);
        telemetry.update();
    }
    @Override
    public void stop() {
    }

    public int avgHeight(){
        runtime.reset();
        int height = 0;
        int loops = 0;
        while(runtime.seconds() < 5) {
            byte[] pixyBytes = pixy.readShort(0x51, 5);
            height = height + pixyBytes[1];
            loops++;
        }
        return height/loops;
    }
}
