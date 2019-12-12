package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

public class Hardware_CamskyBot_V3 extends LinearOpMode {
    private OpenCvCamera webcam;
    private int resWidth = 640, resHeight = 480;
    private Point p1 = new Point(0,0);
    private Point p2 = new Point(resHeight,resWidth);
    private org.firstinspires.ftc.teamcode.StoneDetectorModified stoneDetectorModified = new org.firstinspires.ftc.teamcode.StoneDetectorModified(p1, p2);

    public Hardware_CamskyBot_V3(){}

    public  void Init(HardwareMap hard) {
        stoneDetectorModified.stonesToFind = 1;
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.UPRIGHT);
    }
    @Override
    public void runOpMode() {

    }
}
