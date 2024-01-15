package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class PiracyGood extends LinearOpMode {
    Robot piracyWii=new Robot();
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 75% power at a 25 inch error.   (0.75 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 50% power at a 25 degree Yaw error.   (0.50 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)



    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
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
    private IMU imu;
    private Servo rotate;
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
    boolean red = true;
    double desiredDirection;
    //for forward: all four motors need to be negative

    @Override
    public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        clawl = hardwareMap.get(ServoImplEx.class, "clawl");
        clawr = hardwareMap.get(ServoImplEx.class, "clawr");
        imu = hardwareMap.get(IMU.class, "imu");
        rotate = hardwareMap.get(Servo.class, "rotate");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


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
        clawl.setPosition(.75);
        //close
        clawr.setPosition(0.8);
        rotate.setPosition(0.1);

        if (USE_WEBCAM)
            setManualExposure(1, 255);  // Use low exposure time to reduce motion blur

        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this


        while (gamepad1.left_bumper) {
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
        runtime.reset();
        while (runtime.seconds() < 1 && opModeIsActive()) {
            byte[] pixyBytes1 = pixy.readShort(0x51, 5);
            telemetry.addData("number of Signature 1", pixyBytes1[0]);
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]);
            byte[] pixyBytes2 = pixy.readShort(0x52, 2);
            telemetry.addData("number of Signature 2", pixyBytes2[0]);
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]);
            telemetry.update();
            if (red == true) {
                if (pixyBytes1[1] < 90 && pixyBytes1[1] != 0) {
                    position = 'C';
                } else if (pixyBytes1[1] > 90) {
                    position = 'L';
                } else if (pixyBytes1[0] == 0) {
                    position = 'R';
                }
            }
            if (red == false) {
                if (pixyBytes2[1] > 0) {
                    position = 'L';
                } else if (pixyBytes2[1] < 0) {
                    position = 'C';
                } else if (pixyBytes2[0] == 0) {
                    position = 'R';
                }
            }
        } //close pixy detection time-based while loop

        //Robot needs to drive and move forward like 24in ish
        piracyWii.drive(34, Robot.directions.FORWARD, 0.25);

        //If Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
        if (position == 'L') {
            piracyWii.turn(90, 0.25);
            piracyWii.drive(3, Robot.directions.FORWARD, 0.25);
            //TODO make claw open
            clawr.setPosition(0.9);
            piracyWii.drive(-3, Robot.directions.FORWARD, 0.25);
            piracyWii.turn(0, 0.25);
            //Drive the remaining 48in
            piracyWii.drive(16, Robot.directions.FORWARD, 0.25);
        } else if (position == 'C') {
            // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
            piracyWii.drive(12, Robot.directions.FORWARD, 0.25);
            piracyWii.turn(175, .25);
            //open
            clawr.setPosition(0.9);
            piracyWii.drive(-4, Robot.directions.FORWARD, 0.25);
        } else if (position == 'R') {
            //Then turn right 90 degrees drop pixel at right
            piracyWii.turn(-90,.25);
            piracyWii.drive(3, Robot.directions.FORWARD, .25);
            clawr.setPosition(0.9);
            piracyWii.drive (-3, Robot.directions.FORWARD, .25);
            piracyWii.turn(0,.25);
            //Drive the remaining 48in
            piracyWii.drive(14, Robot.directions.FORWARD, 0.25);
        }
        if (red == true) {
            //Then turn 90 degrees to the right after the 72in
            piracyWii.turn(90,0.25);
        } else if (red == false) {
            piracyWii.turn(-90, 0.25);
        }
        //After that drive forward 96in underneath the stage door
        piracyWii.drive(-70, Robot.directions.FORWARD, 0.25);
        //turn to see tags
        if (red == true) {
            piracyWii.turn(10,.25);
        } else {
            piracyWii.turn(-10,.25);
        }
        //Then april tag will direct robot to backdrop
        targetFound = false;
        desiredTag = null;
        if (red == true && position == 'L') {
            DESIRED_TAG_ID = 4;
        } else if (red == true && position == 'R') {
            DESIRED_TAG_ID = 6;
        } else if (red == true && position == 'C') {
            DESIRED_TAG_ID = 5;
        }
        while (opModeIsActive()) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = false;
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
            //Mitchell not just kinda weird he is weird
            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", -drive, -strafe, turn);
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                break;
            } else {
                if (red == true) {
                    piracyWii.drive(-2, Robot.directions.SIDE, .25);
                } else {
                    piracyWii.drive(2, Robot.directions.SIDE, .25);
                }
                telemetry.addData("\n>", "no target found\n");
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", -drive, -strafe, turn);
            }
            telemetry.update();
        }
        // Apply desired axes motions to the drivetrain.
        moveRobot(-drive, -strafe, turn);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        //PoseStorage.currentPose = drive.getPoseEstimate();

    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  -x +y +yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

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
        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(-leftBackPower);
        rightBack.setPower(-rightBackPower);
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


