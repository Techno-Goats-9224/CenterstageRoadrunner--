package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    public DcMotorEx rightBack;
    public DcMotorEx leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public Servo drone;
    public ServoImplEx clawl;
    public ServoImplEx clawr;
    public DcMotorEx arm;
    public BNO055IMU imu;
    public Servo rotate;
    public Pixy pixy;

    public enum directions{
        FORWARD,
        SIDE
    }

    public boolean init(HardwareMap hardwareMap) {
        drone = hardwareMap.get(Servo.class,"drone");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        clawl = hardwareMap.get(ServoImplEx.class, "clawl");
        clawr = hardwareMap.get(ServoImplEx.class, "clawr");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rotate = hardwareMap.get(Servo.class, "rotate");
        pixy = hardwareMap.get(Pixy.class, "pixy");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawl.setDirection(Servo.Direction.REVERSE);
        clawr.setDirection(Servo.Direction.REVERSE);
        rotate.setDirection(Servo.Direction.REVERSE);

        return true;
    }

    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    double lastParEncoder_in;
    double lastPerpEncoder_in;
    double xPos_in;
    double yPos_in;
    public void drive(double inches, directions dir, double power) {
        lastParEncoder_in = encoderTicksToInches(rightBack.getCurrentPosition());
        lastPerpEncoder_in = encoderTicksToInches(leftFront.getCurrentPosition());
        xPos_in = encoderTicksToInches(rightBack.getCurrentPosition());
        yPos_in = encoderTicksToInches(leftFront.getCurrentPosition());

        if(dir == directions.FORWARD) {
            while (((Math.abs(xPos_in - (inches + lastParEncoder_in))) > 0.5)) {
                xPos_in = encoderTicksToInches(rightBack.getCurrentPosition());
                desiredDirection = (xPos_in - (inches + lastParEncoder_in)) / (Math.abs(xPos_in - (inches + lastParEncoder_in)));

                leftFront.setPower(-desiredDirection * power);
                leftBack.setPower(-desiredDirection * power);
                rightFront.setPower(desiredDirection * power);
                rightBack.setPower(-desiredDirection * power);
            }
        }
        //positive inches is left
        if(dir == directions.SIDE) {
            while (((Math.abs(yPos_in - (inches + lastPerpEncoder_in))) > 0.5)) {
                yPos_in = encoderTicksToInches(leftFront.getCurrentPosition());
                desiredDirection = (yPos_in - (inches + lastPerpEncoder_in)) / (Math.abs(yPos_in - (inches + lastPerpEncoder_in)));

                leftFront.setPower(desiredDirection * power);
                leftBack.setPower(-desiredDirection * power);
                rightFront.setPower(desiredDirection * power);
                rightBack.setPower(desiredDirection * power);
            }
        }
    }
    double heading;
    double desiredDirection;
    public void turn(double degrees, double power) {
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;

        while (((Math.abs(degrees - heading)) > 3) /*&& opModeIsActive()*/) {
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
            desiredDirection = (degrees - heading) / (Math.abs(degrees - heading));

            leftFront.setPower(-desiredDirection * power);
            leftBack.setPower(-desiredDirection * power);
            rightFront.setPower(-desiredDirection * power);
            rightBack.setPower(desiredDirection * power);
        }
    }
    public void launchDrone(){
        drone.setPosition(.5);
    }
    public void dontLaunchDrone(){
        drone.setPosition(.1);
    }
    public void openClawl(){
        clawl.setPosition(.6);
    }
    public void closeClawl(){
        clawl.setPosition(.75);
    }
    public void openClawr(){
        clawr.setPosition(.7);
    }
    public void closeClawr() {
        clawr.setPosition(.6);
    }
    public void armUp(){
        arm.setTargetPosition(-500);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void armDown(){
        arm.setTargetPosition(0);
        arm.setPower(0.5);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void armPower(double power){
        arm.setPower(power);
    }
    public void rotateAustralia(){
        //Cuz their down under
        rotate.setPosition(.5);
    }
    public void rotateTysensPersonality(){
        //Mid
        rotate.setPosition(.6);
    }public void rotateAlaska(){
        //up place
        rotate.setPosition(.9);
    }

    public void telemetry(Telemetry telemetry){
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
    public void stop(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        arm.setPower(0);
    }
}