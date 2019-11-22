package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.PI;

@Autonomous
public class DubiousTest extends LinearOpMode {
    HardwareSkybot_V2 r = new HardwareSkybot_V2( true);
    @Override
    public void runOpMode() {
        r.Init(hardwareMap);
        while(!isStarted()){
            telemetry.addData("encDr", r.encDr);
            telemetry.addData("encSt", r.encSt);
            telemetry.addData("encSp", r.encSp);
            telemetry.update();
        }
        waitForStart();
        //r.fata(500,0.3);
        r.stanga(500,0.5);
        telemetry.addData("encDr", r.encDr);
        telemetry.addData("encSt", r.encSt);
        telemetry.addData("encSp", r.encSp);
        telemetry.addData("calculatedTicks", r.calculatedTicks);
        telemetry.update();
        sleep(3000);
        r.startTh = false;
    }
}
