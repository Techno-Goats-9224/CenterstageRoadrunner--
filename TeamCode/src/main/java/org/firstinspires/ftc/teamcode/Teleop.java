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

//list of controller function here: https://docs.google.com/document/d/1mTPaoFG1fvQqmZDU4-IlvfgqJ6lwRfTvbBcxseAkrCM/edit?usp=sharing
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
    private Servo rotate;
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
            rotate = hardwareMap.get(Servo.class, "rotate");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
            clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
            clawl.setDirection(Servo.Direction.REVERSE);
            clawr.setDirection(Servo.Direction.REVERSE);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

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
                arm.setPower(1);
            }else if(gamepad2.dpad_down){
                arm.setPower(-1);
            }else {
                arm.setPower(0);
            }
            if (gamepad2.cross){
                intakel.setPower(-1);
                intaker.setPower(1);
            }else {
                intakel.setPower(0);
                intaker.setPower(0);
            }
            //left maybe not working, need to test
            if(gamepad2.square) {
                clawl.setPosition(0.6);
            }else {
                //open
                clawl.setPosition(0.45);
            }
            if(gamepad2.left_bumper){
                rotate.setPosition(0.1);
            }else{
                rotate.setPosition(0);
            }
            if(gamepad2.circle) {
                //close
                clawr.setPosition(.05);
            }else {
                //open
                clawr.setPosition(.25);
            }
            if (gamepad2.triangle){
                drone.setPosition(.5);
            }else{
                drone.setPosition(1);
                //1 is when not pushed and .5 is when pushed
                // 1 is holding rubberband .5 is out
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
            telemetry.addData("Front Left Encoder", leftFront.getCurrentPosition());
            telemetry.addData("Front Right Encoder", rightBack.getCurrentPosition());
        }
        @Override
        public void stop() {

        }
    }


