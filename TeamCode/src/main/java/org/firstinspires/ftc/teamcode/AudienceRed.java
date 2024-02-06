
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.degrees;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AudienceRed extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 75% power at a 25 inch error.   (0.75 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 50% power at a 25 degree Yaw error.   (0.50 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)



    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int desiredTagID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    AprilTagProcessor.Builder MitchellKindaWeirdAprilTagProcessor;
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
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;

        while (((Math.abs(degrees - heading)) > 3) && opModeIsActive()) {
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            desiredDirection = (degrees - heading) / (Math.abs(degrees - heading));

            leftFront.setPower(-desiredDirection * power);
            leftBack.setPower(-desiredDirection * power);
            rightFront.setPower(-desiredDirection * power);
            rightBack.setPower(desiredDirection * power);

            telemetry.addData("degrees:", degrees);
            telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
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
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this

        // Initialize the Apriltag Detection process
       // initAprilTag();

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        clawl = hardwareMap.get(ServoImplEx.class, "clawl");
        clawr = hardwareMap.get(ServoImplEx.class, "clawr");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rotate = hardwareMap.get(Servo.class, "rotate");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        if (USE_WEBCAM)
            setManualExposure(6, 255);  // Use low exposure time to reduce motion blur

        while (gamepad1.left_bumper && isStopRequested() == false) {
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
        telemetry.addData("Status", "Initialized");
        telemetry.addData("red side? ", red);
        telemetry.update();

        // Wait for driver to press start
        waitForStart();

        //Pixy look for team prop
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

        //Robot needs to drive and move forward like 24in ish
        drive(30, directions.FORWARD, 0.25);

        //If Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
        if (position == 'L') {
            turn(90, directions.LEFT, 0.25);
            drive(2.5, directions.FORWARD, 0.25);
            //open
            clawr.setPosition(0.7);
            drive(-2.5, directions.FORWARD, 0.25);
            turn(0, directions.RIGHT, 0.25);
            //Drive the remaining 48in
            drive(20, directions.FORWARD, 0.25);
        } else if (position == 'C') {
            // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
            drive(16, directions.FORWARD, 0.25);
            turn(175, directions.LEFT, .25);
            //open
            clawr.setPosition(0.7);
            drive(-4, directions.FORWARD, 0.25);
        } else if (position == 'R') {
            //Then turn right 90 degrees drop pixel at right
            turn(-90, directions.RIGHT, .25);
            drive(1.5, directions.FORWARD, .25);
            //open
            clawr.setPosition(0.7);
            drive(-1.5, directions.FORWARD, .25);
            turn(0, directions.LEFT, .25);
            //Drive the remaining 48in
            drive(20, directions.FORWARD, 0.25);
        }
        if (red == true) {
            //Then turn 90 degrees to the right after the 72in
            //turn(90, directions.SIDE, .25); //tags version
            turn(-90, directions.SIDE, 0.25); //screw tags version original
        } else if (red == false) {
            //turn(-90, directions.SIDE, 0.25); //tags version
            turn(90, directions.SIDE, .25); //screw tags version
        }
        runtime.reset();
        while (runtime.seconds()<5){
            drive(0, directions.FORWARD, 0);
        }
        //After that drive forward 96in underneath the stage door
        //drive(-70, directions.FORWARD, 0.25); //this is the tags version
        drive(83, directions.FORWARD, 0.5); //this is screw tags version
        //open
        clawl.setPosition(0.6); //this is screw tags
        runtime.reset();
        while(runtime.seconds() < 1 && opModeIsActive()){
            telemetry.addData("robot", "dont move");
            drive(0, directions.FORWARD, 0);
        }
        drive(-2, directions.FORWARD,.25); //this is screw tags
        /*//screw tags start comment goes here
        //turn to see tags
        if (red == true) {
            turn(90, directions.RIGHT, .25);
            drive(12, directions.SIDE, .25);
        } else {
            turn(-90, directions.RIGHT, .25);
            drive(-12, directions.SIDE, .25);
        }
        runtime.reset();
        while(runtime.seconds() < 3){
            drive(0, directions.FORWARD, 0);
        }
        //Then april tag will direct robot to backdrop
        targetFound = false;
        desiredTag = null;
        if (red == true && position == 'L') {
            desiredTagID = 4;
        } else if (red == true && position == 'C') {
            desiredTagID = 5;
        } else if (red == true && position == 'R') {
            desiredTagID = 6;
        }
        if (red == false && position == 'L') {
            desiredTagID = 1;
        } else if (red == false && position == 'C') {
            desiredTagID = 2;
        } else if (red == false && position == 'R') {
            desiredTagID = 3;
        }

        //drive and look for tag
        while (desiredTag == null && opModeIsActive()) {
            telemetry.addData("desired id", desiredTagID);
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    //  Check to see if we want to track towards this tag.
                    if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            if (red == true) {
                //drive left - but backward so it seems right
                leftFront.setPower(-0.25);
                leftBack.setPower(0.25);
                rightFront.setPower(-0.25);
                rightBack.setPower(-0.25);
            } else {
                //drive right - but backward so it seems right
                leftFront.setPower(0.25);
                leftBack.setPower(-0.25);
                rightFront.setPower(0.25);
                rightBack.setPower(0.25);
            }
            telemetry.addData("no tag found","drive left");
            telemetry.update();

            //the sample had this, so i left it. idk why
            runtime.reset();
            while(runtime.milliseconds() < 10 && opModeIsActive()){}
        }
        //drive to tag while looking at it
        while (desiredTag != null && desiredTag.ftcPose.range - DESIRED_DISTANCE > 0 && opModeIsActive())
        {
            //targetFound = false;
            //desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    //  Check to see if we want to track towards this tag.
                    if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            // If Left Bumper is being pressed - nah, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                if (red == true) {
                    //drive left - but backward so it seems right
                    leftFront.setPower(-0.25);
                    leftBack.setPower(0.2);
                    rightFront.setPower(-0.25);
                    rightBack.setPower(-0.2);
                } else {
                    //drive right - but backward so it seems right
                    leftFront.setPower(0.2);
                    leftBack.setPower(-0.25);
                    rightFront.setPower(0.2);
                    rightBack.setPower(0.25);
                }
                telemetry.addData("no tag found","drive left");
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(-drive, strafe, turn);
            
            //the sample had this, so i left it. idk why
            runtime.reset();
            while(runtime.milliseconds() < 10 && opModeIsActive()){}
        }
        //end of aprilTag
        drive(-12, directions.SIDE, 0.25);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        //arm up
        arm.setTargetPosition(2500);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(arm.isBusy() == true && opModeIsActive()){
            rightFront.setPower(0);
            leftFront.setPower(0);
            telemetry.addData("arm encoder", arm.getCurrentPosition());
            telemetry.update();
        }

        //up above field
        rotate.setPosition(1);
        runtime.reset();
        while(runtime.seconds() < 2 && opModeIsActive()){}
        //open
        clawl.setPosition(0.6);
        runtime.reset();
        while(runtime.seconds() < 1 && opModeIsActive()){}

        //arm down
        arm.setTargetPosition(20);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(arm.isBusy() == true && opModeIsActive()){
            telemetry.addData("arm encoder", arm.getCurrentPosition());
            telemetry.update();
        }

        if(red == true){
            telemetry.addData("driving", "side after arm");
            telemetry.update();
            drive(12, directions.SIDE, 0.25);

        } else if (red == false) {
            telemetry.addData("driving", "side after arm");
            telemetry.update();
            drive(-12, directions.SIDE, 0.25);
        }
         *///screw tags end comment goes here

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        // Transfer the current pose to PoseStorage so we can use it in field centric TeleOp
        //PoseStorage.currentPose = drive.getPoseEstimate();

        }



public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw; //-.52
        double rightFrontPower   =  -x +y +yaw; //.98
        double leftBackPower     =  x -y +yaw; //-1.04
        double rightBackPower    =  x +y -yaw; //-.46

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        // in our centerstage robot with RB reversed, LF, LB, and RB and negative to go forward
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
/**
 * Initialize the AprilTag processor.
 */
private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        MitchellKindaWeirdAprilTagProcessor = new AprilTagProcessor.Builder();
        MitchellKindaWeirdAprilTagProcessor.setDrawTagID(true);       // Default: true, for all detections.
        MitchellKindaWeirdAprilTagProcessor.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        MitchellKindaWeirdAprilTagProcessor.setDrawAxes(true);        // Default: false.
        MitchellKindaWeirdAprilTagProcessor.setDrawCubeProjection(true);        // Default: false.
        MitchellKindaWeirdAprilTagProcessor.setLensIntrinsics(1170.09, 1170.09, 944.944, 574.055); //weird calibration numbers go here (fx, fy, cx, cy)---------------------------


            aprilTag = MitchellKindaWeirdAprilTagProcessor.build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);


        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
        } else {
            visionPortal = new VisionPortal.Builder()
            .setCamera(BuiltinCameraDirection.BACK)
            .addProcessor(aprilTag)
            .build();
        }
}

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
private void    setManualExposure(int exposureMS, int gain){
        // Wait for the camera to be open, then use the controls

        if(visionPortal==null){
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if(visionPortal.getCameraState()!=VisionPortal.CameraState.STREAMING){
            telemetry.addData("Camera","Waiting");
            telemetry.update();
        while(!isStopRequested()&&(visionPortal.getCameraState()!=VisionPortal.CameraState.STREAMING)){
        sleep(20);
        }
        telemetry.addData("Camera","Ready");
        telemetry.update();
        }
        //this is stupid
        // Set camera controls unless we are stopping.
        if(!isStopRequested())
        {
        ExposureControl exposureControl=visionPortal.getCameraControl(ExposureControl.class);
        if(exposureControl.getMode()!=ExposureControl.Mode.Manual){
        exposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl=visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
        }
        }
        }
