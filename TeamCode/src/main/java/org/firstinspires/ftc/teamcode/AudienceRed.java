
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.IMU;
        import com.qualcomm.robotcore.hardware.PwmControl;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.ServoImplEx;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous
public class AudienceRed extends OpMode {
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

    public void drive(double inches, directions dir) {
        if (dir == directions.FORWARD) {
            while (encoderTicksToInches(leftFront.getCurrentPosition()) > -inches && opModeIsActive()) {
                leftFront.setPower(-.31);
                leftBack.setPower(-.31);
                rightFront.setPower(.31);
                rightBack.setPower(-.31);

                telemetry.addData("front left (para)", leftFront.getCurrentPosition());
                telemetry.update();
            }
        }
        if (dir == directions.BACK) {
            while (encoderTicksToInches(leftFront.getCurrentPosition()) < inches) {
                leftFront.setPower(.31);
                leftBack.setPower(.31);
                rightFront.setPower(-.31);
                rightBack.setPower(.31);
            }
        }
        if (dir == directions.LEFT) {
            while (encoderTicksToInches(rightBack.getCurrentPosition()) > -inches) {
                leftFront.setPower(-.31);
                leftBack.setPower(.31);
                rightFront.setPower(-.31);
                rightBack.setPower(.31);
            }
        }
        if (dir == directions.RIGHT) {
            while (encoderTicksToInches(rightBack.getCurrentPosition()) < inches) {
                leftFront.setPower(.31);
                leftBack.setPower(-.31);
                rightFront.setPower(.31);
                rightBack.setPower(.31);
            }
        }
    }
    public void turn(double degrees, directions dir) {
        if (dir == directions.LEFT) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -degrees) {
                leftFront.setPower(-.31);
                leftBack.setPower(-.31);
                rightFront.setPower(.31);
                rightBack.setPower(-.31);
            }
        }
        if (dir == directions.RIGHT) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < degrees) {
                leftFront.setPower(.31);
                leftBack.setPower(.31);
                rightFront.setPower(-.31);
                rightBack.setPower(.31);
            }
        }
    }
    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        arm = hardwareMap.get(DcMotorEx.class,"arm");
        clawl = hardwareMap.get(ServoImplEx.class,"clawl");
        clawr = hardwareMap.get(ServoImplEx.class,"clawr");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        //Pixy look for team prop
        //Robot needs to drive and move forward like 24in ish
        drive(24, directions.FORWARD);
        //Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
       // turn(90, directions.LEFT);
       // clawl.setPosition(0.4);
       // turn(90, directions.RIGHT);
        // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
        //clawl.setPosition(0.4);
       // drive(24, directions.FORWARD);
        //turn(180, directions.RIGHT);
        //clawl.setPosition(0.4);
        //turn(180, directions.RIGHT);
        //Then turn right 90 degrees
        //turn(90, directions.RIGHT);
        //Drive the remaining 48in
        //drive(48, directions.FORWARD);
        //Then turn 90 degrees to the right after the 72in
       // turn(90, directions.RIGHT);
        //After that drive forward 96in underneath the stage door
       // drive(96, directions.FORWARD);
        //Then turn another 90 degrees to the right
        //turn(90, directions.RIGHT);
        //Then Drive forward 24in
       // drive(24, directions.FORWARD);
        //Then turn another 90 degrees the left
        //turn(90, directions.LEFT);
        //Then april tag will direct robot to backdrop
    }
    @Override
    public void loop() {
        telemetry.addData("Parallel encoder", leftFront.getCurrentPosition());
        telemetry.addData("perpendicular encoder", rightBack.getCurrentPosition());
    }
    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}


