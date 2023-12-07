package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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
    private BNO055IMU imu;
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        if (gamepad2.dpad_up && arm.getCurrentPosition() < 1800) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*arm.setTargetPosition(-5000);
            arm.setPower(-1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setTargetPosition(-5000);*/
            arm.setPower(-1);
        }else if(gamepad2.dpad_down && arm.getCurrentPosition() > 50){
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            /*arm.setTargetPosition(0);
            arm.setPower(1);
            arm.setTargetPosition(0);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
             */
                arm.setPower(0.75);
        } else if(gamepad2.dpad_right && arm.getCurrentPosition() > 700){
            //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setTargetPosition(-2500);
            //arm.setPower(-1);
            //arm.setTargetPosition(-2500);
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(-1);
        }else if(gamepad2.dpad_right && arm.getCurrentPosition() < 600) {
            //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setTargetPosition(-2500);
            //arm.setPower(-1);
            //arm.setTargetPosition(-2500);
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.75);
        }else if(gamepad2.right_bumper){
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(1);
        } else if (gamepad2.right_trigger > 0.1) {
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(-1);
        } else {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0);
        }
        /*if (gamepad2.cross){
            intakel.setPower(-1);
            intaker.setPower(1);
        }else {
            intakel.setPower(0);
            intaker.setPower(0);
        }*/
        if(gamepad2.square) {
            //open
            clawl.setPosition(0.6);
        }else {
            //close
            clawl.setPosition(0.75);
        }
        if(gamepad2.left_bumper){
            //down below field
            rotate.setPosition(0.2);
        }else{
            //flat on field
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
            //open
            //when reversed
            //.55 closed too far
            //.4 closed too far
            //.2 closed too far
            //.05 closed too far
            //.9 didn't close far enough
            //.8 is good
            clawr.setPosition(.9);
        }else {
            //close
            //when reversed
            //.5 closed and trying to go farther
            //0 closed and trying to go farther
            //1 is good
            clawr.setPosition(.8);
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
        telemetry.addData("imu first",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("imu second", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("imu third", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("arm encoder", arm.getCurrentPosition());

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