package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
    public class Teleop extends OpMode {
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
        DcMotor drone1;
        DcMotor drone2;
        @Override
        public void init() {
            leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
            leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
            drone1=hardwareMap.get(DcMotor.class, "drone");
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
            if (gamepad1.a){
                drone1.setPower(1);
            }else {
                drone1.setPower(0);
            }
            double x=gamepad1.left_stick_x;
            double y=-gamepad1.left_stick_y;
            telemetry.addData("Gamepad left x",x);
            telemetry.addData("Gamepad left y",y);

            double ly = gamepad1.left_stick_y * 0.8;
            double lx = -gamepad1.left_stick_x * 0.8;
            double rx = -gamepad1.right_stick_x * 0.8;

            leftFront.setPower(ly + lx + rx);
            leftBack.setPower(ly - lx + rx);
            rightFront.setPower(-ly + lx + rx);
            rightBack.setPower(ly + lx - rx);
            telemetry.addData("leftbackEncoder",leftBack.getCurrentPosition());
            telemetry.addData("leftrontEncoder", leftFront.getCurrentPosition());
            telemetry.addData("rightbackEncoder", rightBack.getCurrentPosition());
            telemetry.addData("rightfrontEncoder", rightFront.getCurrentPosition());
        }
        @Override
        public void stop() {
        }
    }


