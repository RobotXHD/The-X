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
       r.miscare(-125,0,0);
       while(systime + 1000 >  System.currentTimeMillis()){}
       r.miscare(0,0,PIDControllerTestConfig.setpoint);
       while(systime + 1000 > System.currentTimeMillis()){}

    }
}
