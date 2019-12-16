package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Hardware_Cam<ArrayList> extends LinearOpMode {

    private OpenCvCamera webcam;
    private int resWidth = 640, resHeight = 480;
    private Point p1 = new Point(0,0);
    private Point p2 = new Point(resHeight,resWidth);
    public org.firstinspires.ftc.teamcode.StoneDetectorModified stoneDetectorModified = new org.firstinspires.ftc.teamcode.StoneDetectorModified(p1, p2);

    public Hardware_Cam(){}

    public  void Init(HardwareMap hard) {
        stoneDetectorModified.stonesToFind = 1; 
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hard.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hard.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
    }
    public void startDetection(){
        webcam.startStreaming(resWidth, resHeight);
    }
    @Override
    public void runOpMode() {

    }
}
