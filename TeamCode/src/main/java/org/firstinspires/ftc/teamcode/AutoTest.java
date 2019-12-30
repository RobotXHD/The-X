package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTest extends LinearOpMode {
    Hardware_Cam cam = new Hardware_Cam();
    Hardware_Skybot_V3 r = new Hardware_Skybot_V3(true);

    @Override
    public void runOpMode() {
        r.Init(hardwareMap);
        cam.Init(hardwareMap);
        cam.startDetection();
        while (!isStarted()){
            telemetry.addData("X sau Y",cam.stoneDetectorModified.foundRectangles());
            telemetry.addData("Ceva: ", cam.stoneDetectorModified.foundScreenPositions());
            telemetry.addData("Ceva v2: ", cam.stoneDetectorModified.foundPozitionare());
        }
        waitForStart();

    }
}
