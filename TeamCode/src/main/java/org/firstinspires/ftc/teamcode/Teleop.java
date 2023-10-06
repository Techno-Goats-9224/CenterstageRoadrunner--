package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
    public class Teleop extends OpMode {
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
        DcMotor drone1;
        DcMotor drone2;
        @Override
        public void init() {
            leftBack = hardwareMap.get(DcMotor.class,"BL");
            rightBack = hardwareMap.get(DcMotor.class,"BR");
            leftFront = hardwareMap.get(DcMotor.class,"FL");
            rightFront = hardwareMap.get(DcMotor.class,"FR");
            drone1=hardwareMap.get(DcMotor.class, "motor1");
            drone2=hardwareMap.get(DcMotor.class, "motor2");
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
                drone1.setPower(1);
                drone2.setPower(-1);
            }else {
                drone1.setPower(0);
                drone2.setPower(0);}
            double x=gamepad1.left_stick_x;
            double y=-gamepad1.left_stick_y;
            telemetry.addData("Gamepad x4",x);
            telemetry.addData("Gamepad y4",y);

            double ly = gamepad1.left_stick_y * 0.8;
            double lx = -gamepad1.left_stick_x * 0.8;
            double rx = -gamepad1.right_stick_x * 0.8;

            leftFront.setPower(ly + lx + rx);
            leftBack.setPower(ly - lx + rx);
            rightFront.setPower(-ly + lx + rx);
            rightBack.setPower(ly + lx - rx);
        }
        @Override
        public void stop() {
        }
    }


