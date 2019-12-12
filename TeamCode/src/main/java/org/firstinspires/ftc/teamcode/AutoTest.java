package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class AutoTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode(){

       Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);
       r.Init(hardwareMap);
       while(!isStarted()){
           TelemetryPacket telemetryPacket = new TelemetryPacket();
           telemetryPacket.put("EncDr", r.encDr);
           telemetryPacket.put("EncSt", r.encSt);
           dashboard.sendTelemetryPacket(telemetryPacket);
       }
       waitForStart();
       r.gotoY(10000);
       r.rotatie(90);
       r.gotoX(10000);
       r.startTh = false;
    }
}
