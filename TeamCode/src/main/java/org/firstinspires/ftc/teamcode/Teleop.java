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
    private Pixy pixy;
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
        pixy = hardwareMap.get(Pixy.class, "pixy");

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
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*arm.setTargetPosition(-5000);
            arm.setPower(-1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setTargetPosition(-5000);*/
            if(arm.getCurrentPosition() < 2000) {
                arm.setPower(-1);
            } else{
                arm.setPower(0);
            }
        }else if(gamepad2.dpad_down){
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            /*arm.setTargetPosition(0);
            arm.setPower(1);
            arm.setTargetPosition(0);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
             */
            if(arm.getCurrentPosition() > 100) {
                arm.setPower(0.75);
            } else{
                arm.setPower(0);
            }
        } else if(gamepad2.dpad_right){
            //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setTargetPosition(-2500);
            //arm.setPower(-1);
            //arm.setTargetPosition(-2500);
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(arm.getCurrentPosition() < 1200) {
                arm.setPower(-1);
            } else if(arm.getCurrentPosition() > 1500){
                arm.setPower(0.75);
            } else{
                arm.setPower(0);
            }
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
        if(gamepad1.cross){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(gamepad2.square) {
            //open
            clawl.setPosition(0.6);
        }else if(gamepad2.circle) {
            //open
            clawr.setPosition(.7);
        }else if (gamepad2.cross){
            //open
            clawl.setPosition(0.6);
            clawr.setPosition(.7);
        }else {
            //close
            clawl.setPosition(0.75);
            clawr.setPosition(.6);
        }

        if(gamepad2.left_trigger > 0.1){
            //up
            rotate.setPosition(0.4);
        } else if(gamepad2.left_bumper){
            //down below field
            rotate.setPosition(0.2);
        } else{
            //flat on field
            rotate.setPosition(0.3);
        }
        if (gamepad2.triangle){
            drone.setPosition(.5);
        }else{
            drone.setPosition(1);
            //1 is when not pushed and .5 is when pushed
            // 1 is holding rubberband .5 is out
        }

        //normal driving code from gm0.org
        /*double ly = gamepad1.left_stick_y * 0.8;
        double lx = -gamepad1.left_stick_x * 0.8;
        double rx = -gamepad1.right_stick_x * 0.8;

        leftFront.setPower(ly + lx + rx);
        leftBack.setPower(ly - lx + rx);
        rightFront.setPower(-ly + lx + rx);
        rightBack.setPower(ly + lx - rx);
*/

        //gm0.org field centric
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        leftFront.setPower((rotY + rotX + rx) / denominator);
        leftBack.setPower((rotY - rotX + rx) / denominator);
        rightFront.setPower((-rotY + rotX + rx) / denominator);
        rightBack.setPower((rotY + rotX - rx) / denominator);

        /*
        //DYSFUNCTIONAL - field centric driving code from learnroadrunner.com
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -(gamepad1.left_stick_y * 0.75),
                -(gamepad1.left_stick_x * 0.75)
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
         */

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
        byte[] pixyBytes1 = pixy.readShort(0x51, 5); // need this
        telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
        telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this
        byte[] pixyBytes2 = pixy.readShort(0x52, 2); // need this
        telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
        telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this
        telemetry.update();
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