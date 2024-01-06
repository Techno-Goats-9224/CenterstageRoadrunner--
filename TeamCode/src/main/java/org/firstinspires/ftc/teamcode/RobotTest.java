package org.firstinspires.ftc.teamcode;

import android.sax.StartElementListener;

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
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad2.dpad_up){
            piracyWii.armUp();
            telemetry.addData("in up: ", piracyWii.arm.getTargetPosition());
        }
        else if(gamepad2.dpad_down){
            piracyWii.armDown();
            telemetry.addData("in down: ", piracyWii.arm.getTargetPosition());
        } else{
            piracyWii.armPower(0);
            telemetry.addData("in zero: ", piracyWii.arm.getTargetPosition());
        }
        telemetry.addData("arm encoder", piracyWii.arm.getCurrentPosition());
        telemetry.update();

    }
}
