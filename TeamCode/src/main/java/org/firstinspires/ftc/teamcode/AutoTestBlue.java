package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTestBlue extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    //Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        //r.Init(hardwareMap);
        cam.startDetection();
        while (!isStarted()){
            telemetry.addData("X:", cam.stoneDetectorModified.foundScreenPositions().get(0).x);
            telemetry.addData("Y:", cam.stoneDetectorModified.foundScreenPositions().get(0).y);
            if(cam.stoneDetectorModified.foundScreenPositions().get(0).y>143){
                telemetry.addData("Position?", "LEFT");
            }
            else if(cam.stoneDetectorModified.foundScreenPositions().get(0).y>32 ){
                telemetry.addData("Position?", "CENTER");
            }
            else{
                telemetry.addData("Position?", "RIGHT");
            }
            telemetry.update();
            /*telemetry.addData("DR", r.encDr);
            telemetry.addData("ST", r.encSt);
            telemetry.addData("SP", r.encSp);
            telemetry.update();
           /* telemetry.addData("Ceva: ", cam.stoneDetectorModified.foundScreenPositions());
            telemetry.addData("Ceva v2: ", cam.stoneDetectorModified.foundPozitionare());*/
        }
        waitForStart();
        //r.gotoY(500);
    }
}
