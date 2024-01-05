
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
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
        clawl.setPosition(.75);
        //close
        clawr.setPosition(0.8);
        rotate.setPosition(0.1);

        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this


        while(gamepad1.left_bumper) {
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

        runtime.reset();
        while (runtime.seconds()<1 && opModeIsActive()) {
            byte[] pixyBytes1 = pixy.readShort(0x51, 5); // need this
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this
            byte[] pixyBytes2 = pixy.readShort(0x52, 2); // need this
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this
            telemetry.update();
            if(red == true){
                if (pixyBytes1[1] < 90 && pixyBytes1[1] != 0) {
                    position = 'C';
                } else if (pixyBytes1[1] > 90) {
                    position = 'L';
                } else if (pixyBytes1[0] == 0) {
                    position = 'R';
                }
            }
            if(red == false){
                if(pixyBytes2[1] > 0 ){
                    position = 'L';
                } else if (pixyBytes2[1] < 0){
                    position = 'C';
                } else if (pixyBytes2[0] == 0) {
                    position = 'R';
                }
            }
        }

            //Pixy look for team prop
        //Robot needs to drive and move forward like 24in ish

        drive(34, directions.FORWARD, 0.25);

        //If Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
            if (position == 'L'){

                turn(90, directions.LEFT, 0.25);
                drive(3, directions.FORWARD, 0.25);
                clawr.setPosition(0.9);
                drive(-3, directions.FORWARD, 0.25);
            }
            else if (position == 'C'){
                // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
                drive(0, directions.FORWARD, 0.25);
                //open
                clawr.setPosition(0.9);
                drive(-3, directions.FORWARD, 0.25);
            }
            else if(position== 'R'){
                //Then turn right 90 degrees drop pixel at right
                turn(-90, directions.RIGHT, .25);
                drive(3, directions.FORWARD, .25);
                clawr.setPosition(0.9);
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