package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTestRed extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);
    private int caz;
    private double rotatie;
    @Override
    public void runOpMode() {
        cam.Init(hardwareMap);
        r.Init(hardwareMap);
        cam.startDetection();
        while (!isStarted()){
            telemetry.addData("X:", cam.stoneDetectorModified.foundScreenPositions().get(0).x);
            telemetry.addData("Y:", cam.stoneDetectorModified.foundScreenPositions().get(0).y);
            if(cam.stoneDetectorModified.foundScreenPositions().get(0).y>143){
                telemetry.addData("Position", "LEFT");
                caz = -1;
            }
            else if(cam.stoneDetectorModified.foundScreenPositions().get(0).y>32){
                telemetry.addData("Position", "CENTER");
                caz = 0;
            }
            else{
                telemetry.addData("Position", "RIGHT");
                caz = 1;

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
       // cam.stopDetection();
        if(caz == -1){
            r.gotoY(3000,1);
            r.rotatie(45,1);
            r.startColect();
            r.gotoY(20000,1);
            r.Colect(0.3);
            r.gotoY(-7273,1);
            r.rotatie(45,1);
            r.gotoY(-54000,1);
            r.rotatie(90,1);
            r.alinierePlaca(-4500,0.2);
            r.prindrePlate();
            r.startColectReverse();
            sleep(1000);
            r.Colect(-0.3);
            r.gotoX(12000,1,250);
            r.gotoX(-2000,1,250);
            rotatie = r.totalRot;
            r.rotatie(-90,1,10);
            r.gotoY(-5000,1);
            r.desprindrePlate();
            sleep(1000);
            rotatie-=90;
            r.pidRotatie.setSetpoint(rotatie);
            r.gotoX(-5000,1);
            r.gotoY(35000, 1);
            r.rotatie(-45,1);
            r.startColect();
            r.gotoY(8000,1);
            r.Colect(0.3);
            r.gotoY(-8000,1);
            r.rotatie(-135,1);
            r.gotoX(-2000,1);
            r.gotoY(24000,1);
            r.startColectReverse();
            r.gotoY(-10000,1);
        }
        else if(caz == 0){

        }
        else{ //caz == "1"
            r.gotoY(12000, 1);
            r.rotatie(45,1);
            r.startColect();
            r.gotoY(8000,1);
            r.Colect(0.3);
            r.gotoY(-8000,1);
            r.rotatie(45,1);
            r.gotoY(-45000,1);
            r.rotatie(90,1);
            r.alinierePlaca(-4500,0.2);
            r.prindrePlate();
            r.startColectReverse();
            sleep(1000);
            r.Colect(-0.3);
            r.gotoX(12000,1,250);
            r.gotoX(-2000,1,250);
            rotatie = r.totalRot;
            r.rotatie(-90,1,10);
            r.gotoY(-5000,1);
            r.desprindrePlate();
            sleep(1000);
            rotatie-=90;
            r.pidRotatie.setSetpoint(rotatie);
            r.gotoX(-4500,1);
            r.gotoY(26000, 1);
            r.rotatie(-45,1);
            r.startColect();
            r.gotoY(8000,1);
            r.Colect(0.3);
            r.gotoY(-8000,1);
            r.rotatie(-135,1);
            r.gotoX(-2000,1);
            r.gotoY(15000,1);
            r.startColectReverse();
            r.gotoY(-10000,1);
            // r.rotatie(-180,1);
        }
    }
}
