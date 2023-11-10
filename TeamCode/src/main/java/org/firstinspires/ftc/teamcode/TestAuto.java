package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterOpMode extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        runBlocking(shooter.spinUp());
    }
}
class Drive{

}
class Shooter {
    private DcMotorEx motor;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    }

    public Action spinUp() {
        return new Action() {
            @Override
            public void init() {
                motor.setPower(0.8);
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                motor.setPower(0.8);
                double vel = motor.getVelocity();

                packet.put("shooterVelocity", vel);

                return vel < 10_000.0;
            }
        };
    }
}