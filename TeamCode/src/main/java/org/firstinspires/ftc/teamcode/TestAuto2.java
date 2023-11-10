/*package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class TestAuto2 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        MotorControl motorControl = new MotorControl(hardwareMap);
        MotorControlActions motorControlActions = new MotorControlActions(motorControl);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Action traj = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        runBlocking(new RaceParallelCommand(traj, motorControlActions.update()));

    }
}

 */
