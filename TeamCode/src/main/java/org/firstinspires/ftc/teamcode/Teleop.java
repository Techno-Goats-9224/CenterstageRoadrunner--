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

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawl.setDirection(Servo.Direction.REVERSE);
        clawr.setDirection(Servo.Direction.REVERSE);
        rotate.setDirection(Servo.Direction.REVERSE);

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
        //left maybe programmed CR
        if(gamepad2.square) {
            //should be middle
            clawl.setPosition(0.5);
        }else {
            //open
            clawl.setPosition(0.5);
        }
        if(gamepad2.left_bumper){
            rotate.setPosition(0.3);
        }else{
            //when not reversed:
            //.5 was all the way up and trying to go farther
            //.1 was all the way up and trying to go farther
            //1 was all the way  up and trying to go farther
            //when reversed
            //1 was all the way up and trying to go farther
            //0 was all the way down and tryng to go farther
            //.5 was all the way down and trying to go farther
            //.75 was all the way down and trying to go farther
            //.9 was all the way down and trying to go farther
            rotate.setPosition(0.4);
        }
        if(gamepad2.circle) {
            //close
            //when reversed
            //.55 closed too far
            //.4 closed too far
            //.2 closed too far
            //.05 closed too far
            //.9 didn't close far enough
            //.8 is good
            clawr.setPosition(.8);
        }else {
            //open
            //when reversed
            //.5 closed and trying to go farther
            //0 closed and trying to go farther
            //1 is good
            clawr.setPosition(.9);
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

        telemetry.addData("front left power", leftFront.getPower());
        telemetry.addData("front right power", rightFront.getPower());
        telemetry.addData("back left power", leftBack.getPower());
        telemetry.addData("back right power", rightBack.getPower());

        telemetry.addData("left claw servo position", clawl.getPosition());
        telemetry.addData("right claw servo position", clawr.getPosition());
        telemetry.addData("drone servo position", drone.getPosition());
        telemetry.addData("Front Left Encoder (perp) ticks", leftFront.getCurrentPosition());
        telemetry.addData("Negative Back Right Encoder (para) ticks", -rightBack.getCurrentPosition());
        telemetry.addData("Front Left Encoder (perp) inches", encoderTicksToInches(leftFront.getCurrentPosition()));
        telemetry.addData("Negative Back Right Encoder (para) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));
    }
    @Override
    public void stop() {

    }
    public static double X_MULTIPLIER = 0.9787360469; // Multiplier in the X direction: 1.005395271
    public static double Y_MULTIPLIER = 0.982667947; // Multiplier in/ the Y direction
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}