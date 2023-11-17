
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.PwmControl;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.ServoImplEx;


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
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private DcMotorEx arm;
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
        //Pixy look for team prop
        //Robot needs to drive and move forward like 24in ish
        //Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
        //Drop pixel at center: drive past 24in then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
        //Drop pixel right: Turn right 90 degrees then open claw pick up pixel then turn left 90 degrees then drive forward 24in
        //Then turn right 90 degrees
        //Drive the remaining 48in
        //Then turn 90 degrees to the right after the 72in
        //After that drive forward 96in underneath the stage door
        //Then turn another 90 degrees to the right
        //Then Drive forward 24in
        //Then turn another 90 degrees the left
        //Then april tag will direct robot to backdrop
    }
    @Override
    public void loop() {
    }
    @Override
    public void stop() {
    }
}


