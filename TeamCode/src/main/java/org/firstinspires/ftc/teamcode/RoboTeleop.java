package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp()
public class RoboTeleop extends OpMode {
    Robot piracyWii = new Robot();

    @Override
    public void init() {
        piracyWii.init(hardwareMap, telemetry);
        piracyWii.closeClawl();
        piracyWii.closeClawr();
        piracyWii.rotateTysensPersonality();
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //gm0.org driving code
        double ly = gamepad1.left_stick_y * 0.8;
        double lx = -gamepad1.left_stick_x * 0.8;
        double rx = -gamepad1.right_stick_x * 0.8;
        piracyWii.leftFront.setPower(ly + lx + rx);
        piracyWii.leftBack.setPower(ly - lx + rx);
        piracyWii.rightFront.setPower(-ly + lx + rx);
        piracyWii.rightBack.setPower(ly + lx - rx);

        if(gamepad2.dpad_up){
           piracyWii.armUp();
           telemetry.addData("target in up: ", piracyWii.arm.getTargetPosition());
           telemetry.addData("current in up: ", piracyWii.arm.getCurrentPosition());
        }
        else if(gamepad2.dpad_down){
            piracyWii.armDown();
            telemetry.addData("target in down: ", piracyWii.arm.getTargetPosition());
            telemetry.addData("current in down: ", piracyWii.arm.getCurrentPosition());
        } else if(gamepad2.right_trigger > 0.1){
            piracyWii.armPower(0.5);
        } else if(gamepad2.right_bumper){
            piracyWii.armPower(-0.75);
        }else {
            piracyWii.arm.setPower(0);
            telemetry.addData("target in zero: ", piracyWii.arm.getTargetPosition());
            telemetry.addData("current in zero: ", piracyWii.arm.getCurrentPosition());
        }
        if(gamepad2.square){
                piracyWii.openClawl();
            }
        else if(gamepad2.circle){
             piracyWii.openClawr();
        }
        else if(gamepad2.cross){
             piracyWii.openClawr();
             piracyWii.openClawl();
        } else{
             piracyWii.closeClawl();
             piracyWii.closeClawr();
        }
        if(gamepad2.left_trigger>.1) {
                //down below field
                piracyWii.rotateAustralia();
        }else if(gamepad2.left_bumper){
                //up above field
               piracyWii.rotateTysensPersonality();
        }else{
            //flat on field
            piracyWii.rotateAlaska();
        }
        if (gamepad2.triangle){
            piracyWii.launchDrone();
        }else {
            piracyWii.dontLaunchDrone();
        }
        piracyWii.telemetry(this.telemetry);
        telemetry.update();
    }

    @Override
    public void stop(){
        piracyWii.stop();
    }
}
