
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class AudienceBlue extends OpMode {
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private DcMotorEx arm;
    private IMU imu;
    private enum directions {
        FORWARD,
        BACK,
        LEFT,
        RIGHT
    }
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public void drive(double inches, AudienceBlue.directions dir) {
        if (dir == AudienceBlue.directions.FORWARD) {
            while (encoderTicksToInches(rightBack.getCurrentPosition()) < inches) {
                leftFront.setPower(1);
                leftBack.setPower(1);
                rightFront.setPower(1);
                rightBack.setPower(1);
            }
        }
        if (dir == AudienceBlue.directions.BACK) {
            while (encoderTicksToInches(rightBack.getCurrentPosition()) > -inches) {
                leftFront.setPower(-1);
                leftBack.setPower(-1);
                rightFront.setPower(-1);
                rightBack.setPower(-1);
            }
        }
        if (dir == AudienceBlue.directions.LEFT) {
            while (encoderTicksToInches(leftBack.getCurrentPosition()) < inches) {
                leftFront.setPower(-1);
                leftBack.setPower(1);
                rightFront.setPower(1);
                rightBack.setPower(-1);
            }
        }
        if (dir == AudienceBlue.directions.RIGHT) {
            while (encoderTicksToInches(leftBack.getCurrentPosition()) > -inches) {
                leftFront.setPower(1);
                leftBack.setPower(-1);
                rightFront.setPower(-1);
                rightBack.setPower(1);
            }
        }
    }
    public void turn(double degrees, AudienceBlue.directions dir) {
        if (dir == AudienceBlue.directions.LEFT) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -degrees) {
                leftFront.setPower(-1);
                leftBack.setPower(-1);
                rightFront.setPower(1);
                rightBack.setPower(1);
            }
        }
        if (dir == AudienceBlue.directions.RIGHT) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < degrees) {
                leftFront.setPower(1);
                leftBack.setPower(1);
                rightFront.setPower(-1);
                rightBack.setPower(-1);

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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawl.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    // run until the end of the match (driver presses STOP)
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        //Forward 24in
        drive(24, AudienceBlue.directions.FORWARD);
    //Left: Turn left  90 degrees then open claw and pick up pixel turn right 90 degrees
        turn(90, AudienceBlue.directions.LEFT);
        clawl.setPosition(0.4);
        turn(90, AudienceBlue.directions.RIGHT);
    //Center: Open Claw pick up pixel
        clawl.setPosition(0.4);
    //Right: Turn right 90 degrees then open claw and pick up pixel turn left 90 degrees
        turn(90, AudienceBlue.directions.RIGHT);
        clawl.setPosition(0.4);
        turn(90, AudienceBlue.directions.LEFT);
    //Forward 24in
        drive(24, AudienceBlue.directions.FORWARD);
        //Left 90 degrees
        turn(90, AudienceBlue.directions.LEFT);
    //Forward 72 in
        drive(72, AudienceBlue.directions.FORWARD);
    //Then april tag will direct robot to backdrop
    }
    @Override
    public void loop() {
    }
    @Override
    public void stop() {
    }
}


