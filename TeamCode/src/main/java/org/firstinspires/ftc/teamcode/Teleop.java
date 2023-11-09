package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//list of controls here: https://docs.google.com/document/d/1mTPaoFG1fvQqmZDU4-IlvfgqJ6lwRfTvbBcxseAkrCM/edit?usp=sharing
@TeleOp
    public class Teleop extends OpMode {
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx arm;
    private DcMotorEx intakel;
    private DcMotorEx intaker;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private Servo drone;
        @Override
        public void init() {
            intakel = hardwareMap.get(DcMotorEx.class,"intakel");
            intaker = hardwareMap.get(DcMotorEx.class,"intaker");
            arm = hardwareMap.get(DcMotorEx.class,"arm");
            clawl = hardwareMap.get(ServoImplEx.class,"clawl");
            clawr = hardwareMap.get(ServoImplEx.class,"clawr");
            leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
            leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
            drone = hardwareMap.get(Servo.class,"drone");

            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
            clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
            clawl.setDirection(Servo.Direction.REVERSE);

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

            if (gamepad2.dpad_up) {
                arm.setVelocity(1);
            }else if(gamepad2.dpad_down){
                arm.setVelocity(-1);
            }else {
                arm.setVelocity(0);
            }
            if (gamepad2.cross){
                intakel.setPower(-1);
                intaker.setPower(1);
            }else {
                intakel.setPower(0);
                intaker.setPower(0);
            }
            if(gamepad2.square) {
                clawl.setPosition(.6);
            }else {
                clawl.setPosition(0.4);
            }
            if(gamepad2.circle) {
                clawr.setPosition(.25);
            }else {
                clawr.setPosition(.05);
            }
            if (gamepad2.triangle){
                drone.setPosition(.5);
            }else{
                drone.setPosition(1);
            }

            double ly = gamepad1.left_stick_y * 0.8;
            double lx = -gamepad1.left_stick_x * 0.8;
            double rx = -gamepad1.right_stick_x * 0.8;

            leftFront.setPower(ly + lx + rx);
            leftBack.setPower(ly - lx + rx);
            rightFront.setPower(-ly + lx + rx);
            rightBack.setPower(ly + lx - rx);

            telemetry.addData("left claw servo position", clawl.getPosition());
            telemetry.addData("right claw servo position", clawr.getPosition());
            telemetry.addData("drone servo position", drone.getPosition());


        }
        @Override
        public void stop() {

        }
    }


