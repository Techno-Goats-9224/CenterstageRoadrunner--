package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;




    @TeleOp
    public class sidewaysClawTest extends OpMode {
        Servo servo;
        DcMotor motor1;
        DcMotor motor2;
        @Override
        public void init() {
            servo=hardwareMap.get(Servo.class,"servo");
            motor1=hardwareMap.get(DcMotor.class, "motor1");
            motor2=hardwareMap.get(DcMotor.class, "motor2");
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
            if (gamepad1.left_trigger>0.1){
                motor1.setPower(1);
                motor2.setPower(-1);
            }else {
                motor1.setPower(0);
                motor2.setPower(0);}
            if (gamepad1.a) {
                servo.setPosition(0);
                telemetry.addData("servo position", servo.getPosition());
            } else {
                servo.setPosition(.6);
                telemetry.addData("servo", servo.getPosition());

            }
        }
        @Override
        public void stop() {
        }
    }


