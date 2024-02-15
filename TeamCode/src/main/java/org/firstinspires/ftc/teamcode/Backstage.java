package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class Backstage extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private DcMotorEx arm;
    private BNO055IMU imu;
    private Servo rotate;
    private enum directions{
        FORWARD,
        SIDE,
        LEFT,
        RIGHT
    }
    public static double X_MULTIPLIER = 0.9787360469; // Multiplier in the X direction: 1.005395271
    public static double Y_MULTIPLIER = 0.982667947; // Multiplier in/ the Y direction
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    char position;
    ElapsedTime runtime = new ElapsedTime();
    private Pixy pixy; // need this
    double lastParEncoder_in = 0;
    double lastPerpEncoder_in = 0;
    double lastHeading = 0;
    double xPos_in;
    double yPos_in;
    double heading;
    boolean red = false;
    double desiredDirection;
    //for forward: all four motors need to be negative
    public void drive(double inches, directions dir, double power) {
        lastParEncoder_in = encoderTicksToInches(rightBack.getCurrentPosition());
        lastPerpEncoder_in = encoderTicksToInches(leftFront.getCurrentPosition());
        xPos_in = encoderTicksToInches(rightBack.getCurrentPosition());
        yPos_in = encoderTicksToInches(leftFront.getCurrentPosition());

        if(dir == directions.FORWARD) {
            while (((Math.abs(xPos_in - (inches + lastParEncoder_in))) > 0.5) && opModeIsActive()) {
                xPos_in = encoderTicksToInches(rightBack.getCurrentPosition());
                desiredDirection = (xPos_in - (inches + lastParEncoder_in)) / (Math.abs(xPos_in - (inches + lastParEncoder_in)));

                leftFront.setPower(-desiredDirection * power);
                leftBack.setPower(-desiredDirection * power);
                rightFront.setPower(desiredDirection * power);
                rightBack.setPower(-desiredDirection * power);

                telemetry.addData("Front Left Encoder (perp) ticks", leftFront.getCurrentPosition());
                telemetry.addData("Negative Back Right Encoder (para) ticks", -rightBack.getCurrentPosition());
                telemetry.addData("Front Left Encoder (perp) inches", encoderTicksToInches(leftFront.getCurrentPosition()));
                telemetry.addData("Negative Back Right Encoder (para) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));

                telemetry.addData("offset from position:", xPos_in - (inches + lastParEncoder_in));
                telemetry.update();
            }
        }
        //positive inches is left
        if(dir == directions.SIDE) {
            while (((Math.abs(yPos_in - (inches + lastPerpEncoder_in))) > 0.5) && opModeIsActive()) {
                yPos_in = encoderTicksToInches(leftFront.getCurrentPosition());
                desiredDirection = (yPos_in - (inches + lastPerpEncoder_in)) / (Math.abs(yPos_in - (inches + lastPerpEncoder_in)));

                leftFront.setPower(desiredDirection * power);
                leftBack.setPower(-desiredDirection * power);
                rightFront.setPower(desiredDirection * power);
                rightBack.setPower(desiredDirection * power);

                telemetry.addData("Front Left Encoder (perp) ticks", leftFront.getCurrentPosition());
                telemetry.addData("Front Left Encoder (perp) inches", encoderTicksToInches(leftFront.getCurrentPosition()));

                telemetry.addData("offset from position:", yPos_in - (inches + lastPerpEncoder_in));
                telemetry.update();
            }
        }
    }
    //to rotate counterclockwise (increasing imu) RB should be the only negative power
    //positive degrees is clockwise
    public void turn(double degrees, directions dir, double power) {
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        runtime.reset();

        while (((Math.abs(degrees - heading)) > 3) && opModeIsActive()) {
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            desiredDirection = (degrees - heading) / (Math.abs(degrees - heading));

            leftFront.setPower(-desiredDirection * power);
            leftBack.setPower(-desiredDirection * power);
            rightFront.setPower(-desiredDirection * power);
            rightBack.setPower(desiredDirection * power);

            telemetry.addData("degrees:", degrees);
            telemetry.addData("heading", heading);
            telemetry.addData("desired direction", desiredDirection);
            telemetry.addData("offset from position:", degrees - heading);
            telemetry.update();
        }
        /*if (dir == directions.LEFT) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle< degrees && opModeIsActive()) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(-power);

                telemetry.addData("Yaw degrees: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.update();
            }
        }
        if (dir == directions.RIGHT) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle > -degrees && opModeIsActive()) {
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(power);

                telemetry.addData("Yaw degrees: ",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.update();
            }
        }*/
    }
    @Override
    public void runOpMode() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)


        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        arm = hardwareMap.get(DcMotorEx.class,"arm");
        clawl = hardwareMap.get(ServoImplEx.class,"clawl");
        clawr = hardwareMap.get(ServoImplEx.class,"clawr");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rotate = hardwareMap.get(Servo.class, "rotate");

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
        //close
        clawl.setPosition(0.7);
        clawr.setPosition(0.6);
        rotate.setPosition(0.8);

        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this

        //voodoo
        while(gamepad1.left_bumper && !isStopRequested()) {
            telemetry.addData("x or square: ", "blue");
            telemetry.addData("b or circle: ", "red");
            telemetry.update();
            if (gamepad1.x) {
                red = false;
            }
            if (gamepad1.b) {
                red = true;
            }
        }
        telemetry.addData("red side? ", red);
        byte[] pixyBytes1 = pixy.readShort(0x51, 5); // need this
        int byte1Avg = 0;
        byte[] pixyBytes2 = pixy.readShort(0x52, 2); // need this
        int byte2Avg = 0;
        byte[] pixyBytes3 = pixy.readShort(0x53, 2); // need this
        int byte3Avg = 0;
        byte[] pixyBytes4 = pixy.readShort(0x54, 5); // need this
        int byte4Avg = 0;
        byte[] pixyBytes5 = pixy.readShort(0x55, 2); // need this
        int byte5Avg = 0;
        byte[] pixyBytes6 = pixy.readShort(0x56, 2); // need this
        int byte6Avg = 0;
        for (int i = 0; i < 20; i++) {
            pixyBytes1 = pixy.readShort(0x51); // need this
            byte1Avg = byte1Avg + pixyBytes1[1];
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this

            pixyBytes2 = pixy.readShort(0x52);
            byte2Avg = byte2Avg + pixyBytes2[1];
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this


            pixyBytes3 = pixy.readShort(0x53);
            byte3Avg = byte3Avg + pixyBytes3[1];
            telemetry.addData("number of Signature 3", pixyBytes3[0]); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3[1]); // need this


            pixyBytes4 = pixy.readShort(0x54); // need this
            byte4Avg = byte4Avg + pixyBytes4[1];
            telemetry.addData("number of Signature 4", pixyBytes4[0]); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4[1]); // need this

            pixyBytes5 = pixy.readShort(0x55);
            byte5Avg = byte5Avg + pixyBytes5[1];
            telemetry.addData("number of Signature 5", pixyBytes5[0]); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5[1]); // need this


            pixyBytes6 = pixy.readShort(0x56);
            byte6Avg = byte6Avg + pixyBytes6[1];
            telemetry.addData("number of Signature 6", pixyBytes6[0]); // need this
            telemetry.addData("x position of largest block of sig 6", pixyBytes6[1]); // need this
        } //close pixy detection for loop

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for driver to press start
        waitForStart();


        //Pixy look for team prop
        byte1Avg = 0;
        byte2Avg = 0;
        byte3Avg = 0;
        byte4Avg = 0;
        byte5Avg = 0;
        byte6Avg = 0;
        for (int i = 0; i < 20; i++) {
            pixyBytes1 = pixy.readShort(0x51, 5); // need this
            byte1Avg = byte1Avg + pixyBytes1[1];
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this

            pixyBytes2 = pixy.readShort(0x52, 2);
            byte2Avg = byte2Avg + pixyBytes2[1];
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this


            pixyBytes3 = pixy.readShort(0x53, 2);
            byte3Avg = byte3Avg + pixyBytes3[1];
            telemetry.addData("number of Signature 3", pixyBytes3[0]); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3[1]); // need this


            pixyBytes4 = pixy.readShort(0x54, 5); // need this
            byte4Avg = byte4Avg + pixyBytes4[1];
            telemetry.addData("number of Signature 4", pixyBytes4[0]); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4[1]); // need this

            pixyBytes5 = pixy.readShort(0x55, 2);
            byte5Avg = byte5Avg + pixyBytes5[1];
            telemetry.addData("number of Signature 5", pixyBytes5[0]); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5[1]); // need this


            pixyBytes6 = pixy.readShort(0x56, 2);
            byte6Avg = byte6Avg + pixyBytes6[1];
            telemetry.addData("number of Signature 6", pixyBytes6[0]); // need this
            telemetry.addData("x position of largest block of sig 6", pixyBytes6[1]); // need this

            telemetry.update();
            if (red == true) {
                if (byte1Avg < 0) {
                    position = 'C';
                } else if (byte1Avg > 0) {
                    position = 'L';
                } else {
                    position = 'R';
                }
            }
            if (red == false) {
                if (byte2Avg > 0) {
                    position = 'L';
                } else if (byte2Avg < 0) {
                    position = 'C';
                } else {
                    position = 'R';
                }
            }
        } //close pixy detection for loop
        runtime.reset();
        while(runtime.seconds() < 5 && opModeIsActive()) {}

        //Robot needs to drive and move forward like 24in ish
        drive(32, directions.FORWARD, 0.25);

        //If Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
            if (position == 'L'){
                turn(90, directions.LEFT, 0.25);
                drive(1.5, directions.FORWARD, 0.25);
                clawr.setPosition(0.7);
                drive(-3, directions.FORWARD, 0.25);
            }
            else if (position == 'C'){
                // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
                drive(-1, directions.FORWARD, 0.25);
                //open
                clawr.setPosition(0.7);
                drive(-4, directions.FORWARD, 0.25);
            }
            else if(position== 'R'){
                //Then turn right 90 degrees drop pixel at right
                turn(-90, directions.RIGHT, .25);
                drive(1.5, directions.FORWARD, .25);
                clawr.setPosition(0.7);
                drive(-3, directions.FORWARD, .25);
            }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        //PoseStorage.currentPose = drive.getPoseEstimate();

    } //close runOpMode()
} // close entire class
