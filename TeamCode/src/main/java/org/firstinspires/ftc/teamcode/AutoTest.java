package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class AutoTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double systime;
   @Override
    public void runOpMode() throws InterruptedException {

       Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);
       r.Init(hardwareMap);
       systime = System.currentTimeMillis();
       waitForStart();
       r.Y(PIDControllerTestConfig.setpointY);
       sleep(1000);
    }
}
