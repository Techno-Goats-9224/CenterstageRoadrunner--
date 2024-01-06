package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp()
public class RobotTest extends OpMode {
    Robot piracyWii = new Robot();
    @Override
    public void init() {
        piracyWii.init(hardwareMap);
        piracyWii.closeClawl();
        piracyWii.closeClawr();
    }

    @Override
    public void loop() {
        if(gamepad2.dpad_up){
            piracyWii.armUp();
        }
        else if(gamepad2.dpad_down){
            piracyWii.armDown();
        }
    }
}
