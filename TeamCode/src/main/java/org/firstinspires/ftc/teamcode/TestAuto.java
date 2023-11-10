package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ActionOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterOpMode extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake();

        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Action traj = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                .build();
        
        runBlocking(intakel.spinUp());
    }
}
class Intake {
    private DcMotorEx intakel;
    private DcMotorEx intaker;

    public Arm(String configName) {
        intakel = hardwareMap.get(DcMotorEx.class, "intakel");
        intaker = hardwareMap.get(DcMotorEx.class, "intaker");
        
    }

    public Action spinUp() {
        return new Action() {
            @Override
            public void init() {
                
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                intakel.setPower(0.8);
                intaker.setPower(-0.8);
                
                double velL = intakel.getVelocity();
                double velR = intaker.getVelocity();

                packet.put("IntakeLVelocity", velL);
                packet.put("IntakeRVelocity", velR);

                return vel < 10_000.0;
            }
        };
    }
}
