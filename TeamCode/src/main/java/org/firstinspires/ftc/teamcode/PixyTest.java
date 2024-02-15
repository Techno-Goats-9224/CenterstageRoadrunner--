package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PixyTest extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    private Pixy pixy; // need this

    int position = 0;
    boolean red = false;

    @Override
    public void init() {
        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // run until the end of the match (driver presses STOP)
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //Pixy look for team prop
        byte[] pixyBytes1 = pixy.readShort(0x51, 5); // need this
        int byte1Avg = 0;
        byte[] pixyBytes2 = pixy.readShort(0x52, 2); // need this
        int byte2Avg = 0;
        byte[] pixyBytes3 = pixy.readShort(0x53, 2); // need this
        int byte3Avg = 0;
        byte[] pixyBytes4 = pixy.readShort(0x54, 5); // need this
        int byte4Avg = 0;
        byte[] pixyBytes5 = pixy.readShort(0x55, 2); // need this
        int byte5Avg = 0;
        byte[] pixyBytes6 = pixy.readShort(0x56, 2); // need this
        int byte6Avg = 0;
        for (int i = 0; i < 20; i++) {
            pixyBytes1 = pixy.readShort(1); // need this
            byte1Avg = byte1Avg + pixyBytes1[1];
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this

            pixyBytes2 = pixy.readShort(2);
            byte2Avg = byte2Avg + pixyBytes2[1];
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this


            pixyBytes3 = pixy.readShort(3);
            byte3Avg = byte3Avg + pixyBytes3[1];
            telemetry.addData("number of Signature 3", pixyBytes3[0]); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3[1]); // need this


            pixyBytes4 = pixy.readShort(4); // need this
            byte4Avg = byte4Avg + pixyBytes4[1];
            telemetry.addData("number of Signature 4", pixyBytes4[0]); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4[1]); // need this

            pixyBytes5 = pixy.readShort(5);
            byte5Avg = byte5Avg + pixyBytes5[1];
            telemetry.addData("number of Signature 5", pixyBytes5[0]); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5[1]); // need this


            pixyBytes6 = pixy.readShort(6);
            byte6Avg = byte6Avg + pixyBytes6[1];
            telemetry.addData("number of Signature 6", pixyBytes6[0]); // need this
            telemetry.addData("x position of largest block of sig 6", pixyBytes6[1]); // need this

            telemetry.update();
            if (red == true) {
                if (byte1Avg < 0) {
                    position = 'C';
                } else if (byte1Avg > 0) {
                    position = 'L';
                } else {
                    position = 'R';
                }
            }
            if (red == false) {
                if (byte2Avg > 0) {
                    position = 'L';
                } else if (byte2Avg < 0) {
                    position = 'C';
                } else {
                    position = 'R';
                }
            }
        } //close pixy detection for loop

    }
    @Override
    public void stop() {
    }
}
