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
    ElapsedTime runtime = new ElapsedTime();
    private I2cDeviceSynch pixy;

    int starterStack = 0;
    @Override
    public void init() {
        pixy = hardwareMap.get(I2cDeviceSynch.class, "pixy");

        pixy.setI2cAddress(I2cAddr.create7bit(0x01));
        pixy.engage();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // run until the end of the match (driver presses STOP)
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        byte[] pixyBytes = pixy.read(0x51, 5);
        runtime.reset();
        if (pixyBytes[0] == 0 && runtime.seconds() < 1){
            starterStack = 0;
            telemetry.addData("starterStack", starterStack);
            telemetry.addData("Pixy Yes/No", pixyBytes[0]);
            telemetry.update();
        } else if (pixyBytes[1] >= 24){
            starterStack = 4;
            telemetry.addData("starterStack", starterStack);
            telemetry.addData("Pixy Height", pixyBytes[1]);
            telemetry.update();
        } else if (pixyBytes[1] < 24){
            starterStack = 1;
            telemetry.addData("starterStack", starterStack);
            telemetry.addData("Pixy Height", pixyBytes[1]);
            telemetry.update();
        }
    }
        @Override
        public void loop() {
        }
        @Override
        public void stop() {
        }
    }
