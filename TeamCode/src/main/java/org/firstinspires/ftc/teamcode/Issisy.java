package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp

public class Issisy extends OpMode {
    private DcMotorEx arm;
    private Servo leftClaw;
    private Servo rightClaw;


    @Override
    public void init() {
        arm=hardwareMap.get(DcMotorEx.class,"arm");
        leftClaw=hardwareMap.get(Servo.class,"clawl");
        rightClaw=hardwareMap.get(Servo.class,"clawr");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // run until the end of the match (driver presses STOP)

    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        if (gamepad2.cross == true) {
            leftClaw.setPosition(0.6);

        } else {
            leftClaw.setPosition(0.7);
        }
        if (gamepad2.square == true) {
            rightClaw.setPosition(0.6);
        } else {
            rightClaw.setPosition(0.5);
        }
    }
    @Override
    public void loop() {
    }
    @Override
    public void stop() {
    }
}


