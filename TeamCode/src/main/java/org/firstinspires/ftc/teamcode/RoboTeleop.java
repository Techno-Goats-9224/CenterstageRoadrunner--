package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class RoboTeleop extends OpMode {
    Robot piracyWii = new Robot();
    @Override
    public void init() {
        piracyWii.init(hardwareMap);
        piracyWii.closeClawl();
        piracyWii.closeClawr();
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad2.dpad_up){
           piracyWii.armUp();
        }
        else if(gamepad2.dpad_down){
            piracyWii.armDown();
        } else {
            piracyWii.armPower(0);
            telemetry.addData("in zero: ", piracyWii.arm.getTargetPosition());
         if(gamepad2.square){
                piracyWii.openClawl();
            }
         else if(gamepad2.circle){
             piracyWii.openClawr();
         }
         else if(gamepad2.cross){
             piracyWii.openClawr();
             piracyWii.openClawl();
         } else{
             piracyWii.closeClawl();
             piracyWii.closeClawr();
         }
            if(gamepad2.left_trigger>.1) {
                //down below field
                piracyWii.rotateAustralia();
            }else if(gamepad2.left_bumper){
                //up above field
               piracyWii.rotateTysensPersonality();
            }else{
                //flat on field
                piracyWii.rotateAlaska();
            }
            if (gamepad2.triangle){
                piracyWii.launchDrone();
            }else {
               piracyWii.dontlaunchDrone();
            }


        }
        telemetry.addData("arm encoder", piracyWii.arm.getCurrentPosition());
        telemetry.update();

    }
}
