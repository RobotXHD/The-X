package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous

public class AutoSkystone extends LinearOpMode {
    private OpenCvCamera webcam;
    private int resWidth = 640, resHeight = 480;
    private Point p1 = new Point(0,0);
    private Point p2 = new Point(resHeight,resWidth);
    private StoneDetectorModified stoneDetectorModified = new StoneDetectorModified(p1, p2);

    @Override
    public void runOpMode() {
        stoneDetectorModified.stonesToFind = 6;
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.UPRIGHT);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("X sau Y: ", stoneDetectorModified.foundRectangles());
            telemetry.addData("Ceva: ", stoneDetectorModified.foundScreenPositions());
            telemetry.addData("Ceva: ", stoneDetectorModified.foundPozitionare());
            telemetry.update();
        }
    }
}