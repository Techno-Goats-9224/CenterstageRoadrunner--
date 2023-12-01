
package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)



    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
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
    private enum directions{
        FORWARD,
        BACK,
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
    ElapsedTime runtime = new ElapsedTime();
    private Pixy pixy; // need this
    double lastParEncoder = 0;
    double lastPerpEncoder = 0;
    boolean red = true;
    public void drive(double inches, directions dir, double power) {
        lastParEncoder = encoderTicksToInches(rightBack.getCurrentPosition());
        lastPerpEncoder = encoderTicksToInches(leftFront.getCurrentPosition());
        if (dir == directions.FORWARD) {
            while (encoderTicksToInches(rightBack.getCurrentPosition()) > -inches + lastParEncoder && opModeIsActive()) {
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(power);
                rightBack.setPower(-power);

                telemetry.addData("Front Left Encoder (perp) ticks", leftFront.getCurrentPosition());
                telemetry.addData("Negative Back Right Encoder (para) ticks", -rightBack.getCurrentPosition());
                telemetry.addData("Front Left Encoder (perp) inches", encoderTicksToInches(leftFront.getCurrentPosition()));
                telemetry.addData("Negative Back Right Encoder (para) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));

                telemetry.addData("Desired position:", lastParEncoder + -inches);
                telemetry.update();
            }
        }
        if (dir == directions.BACK) {
            while (encoderTicksToInches(rightBack.getCurrentPosition()) < inches + lastParEncoder && opModeIsActive()) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(-power);
                rightBack.setPower(power);

                telemetry.addData("Negative Back Right Encoder (para) ticks", -rightBack.getCurrentPosition());
                telemetry.addData("Negative Back Right Encoder (para) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));
            }
        }
        if (dir == directions.LEFT) {
            while (encoderTicksToInches(leftFront.getCurrentPosition()) > -inches + lastPerpEncoder && opModeIsActive()) {
                leftFront.setPower(power);
                leftBack.setPower(-power);
                rightFront.setPower(power);
                rightBack.setPower(power);
            }
        }
        if (dir == directions.RIGHT) {
            while (encoderTicksToInches(leftFront.getCurrentPosition()) < inches + lastPerpEncoder && opModeIsActive()) {
                leftFront.setPower(-power);
                leftBack.setPower(power);
                rightFront.setPower(-power);
                rightBack.setPower(power);
            }
        }
    }
    public void turn(double degrees, directions dir, double power) {
        if (dir == directions.LEFT) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < degrees && opModeIsActive()) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(-power);

                telemetry.addData("Yaw degrees: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
        if (dir == directions.RIGHT) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -degrees && opModeIsActive()) {
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(power);

                telemetry.addData("Yaw degrees: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
    }
    @Override
    public void runOpMode() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        arm = hardwareMap.get(DcMotorEx.class,"arm");
        clawl = hardwareMap.get(ServoImplEx.class,"clawl");
        clawr = hardwareMap.get(ServoImplEx.class,"clawr");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawl.setDirection(Servo.Direction.REVERSE);


        if (USE_WEBCAM)
            setManualExposure(1, 255);  // Use low exposure time to reduce motion blur

        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for driver to press start

        waitForStart();

        runtime.reset();
        while (runtime.seconds()<5 && opModeIsActive()) {
            byte[] pixyBytes1 = pixy.readShort(0x51, 5); // need this
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this
            byte[] pixyBytes2 = pixy.readShort(0x52, 2); // need this
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this
            telemetry.update();
        }
        //Pixy look for team prop
        //Robot needs to drive and move forward like 24in ish
        drive(30, directions.FORWARD, 0.25);

        //Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
        turn(70, directions.LEFT, 0.25);
        // clawl.setPosition(0.4);
        turn(0, directions.RIGHT, 0.25);

        // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
        //clawl.setPosition(0.4);
        // drive(24, directions.FORWARD);
        //turn(180, directions.RIGHT);
        //clawl.setPosition(0.4);
        //turn(180, directions.RIGHT);
        //Then turn right 90 degrees
        //turn(90, directions.RIGHT);

        //Drive the remaining 48in
        if (red = true) {
            drive(18, directions.FORWARD, 0.25);
        }
        else {
            turn(85, directions.BACK, 0.25);
        }
        //Then turn 90 degrees to the right after the 72in

        if (red = true) {
            turn(85, directions.LEFT, 0.25);
        }
        else {
            turn(85, directions.RIGHT, 0.25);
        }
        // In blue has to turn other way
        //After that drive forward 96in underneath the stage door
        drive(70,directions.BACK, 0.25);
        drive(24,directions.LEFT, 0.50);
        // Opposite for blue
        //Then Drive forward 24in
        // drive(24, directions.FORWARD);
        //Then turn another 90 degrees the left
        //turn(90, directions.LEFT);
        runtime.reset();
        while (runtime.seconds()<5 && opModeIsActive()) {
            //Then april tag will direct robot to backdrop
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
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

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }


        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

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

