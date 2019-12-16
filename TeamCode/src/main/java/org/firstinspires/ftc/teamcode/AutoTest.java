package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);

    @Override
    public void runOpMode() {
        //Hardware_Cam cam = new Hardware_Cam();
        r.Init(hardwareMap);
        //cam.Init(hardwareMap);
        while (!isStarted()) {

            TelemetryPacket telemetryPacket = new TelemetryPacket();
            telemetryPacket.put("EncDr: ", r.encDr);
            telemetryPacket.put("EncSt: ", r.encSt);
            telemetryPacket.put("PIDR", r.correctionR);
            telemetryPacket.put("PIDY", r.correctionY);
            telemetryPacket.put("PIDX", r.correctionX);
            dashboard.sendTelemetryPacket(telemetryPacket);
        }
        waitForStart();
        r.gotoY(10000);
        r.rotatie(-45);
        r.gotoY(5000);
        r.gotoY(-7000);
        r.rotatie(135);
        r.gotoY(20000);

    }
}
