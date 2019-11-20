package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;

@Autonomous
public class AutonomEsuat extends LinearOpMode {
    private OpenCvCamera webcam;
    private int resWidth = 800, resHeight = 448;
    private Point p1 = new Point(resHeight * 0.55,0);
    private Point p2 = new Point(resHeight,resWidth);
    private StoneDetectorModified stoneDetectorModified = new StoneDetectorModified(p1, p2);
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;
    private double forward, right, clockwise, tempforward, tempright;
    double lastCorrectedY = 0, lastCorrectedX = 0, correctedY = 0, correctedX = 0, deltaY, deltaX,sindeltay,sindeltax, cosdeltay,cosdeltax;
    double currentY, currentX, encRot;
    volatile double angle; // sens trigonometric +
    double d = 377, omniLengthMm = 188.49555921538759430775860299677, mmPerTick = omniLengthMm / 4000, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLengthMm / 360) * 4000;
    long EncSp, EncSt, EncDr, lastEncDr;
    boolean stop = false;
    volatile boolean encodereCitite = false;
    long timestamp, lastTimestamp,LAST;
    private ExpansionHubMotor motorss, motorsf, motords, motordf;
    double targetAngle = 0, targetDist, arctg, calcPower;
    double kp = 1, kp2 = 0.35;
    private Thread Loc = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while(!isStopRequested()){
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorCurrentPosition(encoderSpate);
                encDrVal = bulkData.getMotorCurrentPosition(encoderDreapta);
                encStVal = bulkData.getMotorCurrentPosition(encoderStanga);
                EncSp = -encSpVal;
                EncSt = -encStVal;
                EncDr = -encDrVal;

                encRot = ((EncDr - EncSt) / 2.0);
                angle = Math.toRadians(encRot / tickPerDeg) % (2 * PI);

                lastCorrectedY = correctedY;
                lastCorrectedX = correctedX;

                correctedY = (EncSt + EncDr)/2.0;
                correctedX = EncSp + 0.656 * encRot;

                deltaY = correctedY - lastCorrectedY;
                deltaX = correctedX - lastCorrectedX;

                cosdeltay = (Math.cos(angle) * deltaY);
                cosdeltax = (Math.cos((PI/2)- angle) * deltaX);
                sindeltay = -(Math.sin(angle) * deltaY);
                sindeltax = (Math.sin((PI/2)-angle) * deltaX);

                currentY = currentY + (cosdeltay + cosdeltax) * mmPerTick;
                currentX = currentX + (sindeltay + sindeltax) * mmPerTick;
            }
        }
    });

    public void calcule(double X, double Y){
        double dx = X - currentX;
        double dy = Y - currentY;

        if(dx > 0){
            if(dy > 0){
                //1
                arctg = (Math.atan(dx / dy));
            }
            else{
                //2
                arctg = (Math.atan(dx / dy) + PI);
            }
        }
        else{
            if(dy > 0){
                //3
                arctg = (Math.atan(dx / dy) + 2*PI);
            }
            else{

                //4
                arctg = (Math.atan(dx / dy) + PI);
            }
        }
        targetAngle = arctg + angle;
        if(targetAngle < PI){
            targetAngle = -targetAngle;
        }
        else{
            targetAngle = PI - (targetAngle - PI);
        }
        //targetDist = Math.sqrt((X - currentX) * (X - currentX) + (Y - currentY) * (Y - currentY));
        /*
        if(targetAngle > 0){
            while(targetAngle > 0){
                targetAngle = angle + arctg;
                calcPower= -(targetAngle * kp); // cu +
                if(calcPower > 1){
                    calcPower = 1;
                }
                power(-calcPower, -calcPower, calcPower, calcPower);
                telemetry.addData("targetAngle", targetAngle);
                telemetry.update();
            }
        }
        else{
            while(targetAngle < 0){
                targetAngle = angle + arctg;
                calcPower= -(targetAngle * kp); // cu +
                if(calcPower > 1){
                    calcPower = 1;
                }
                power(-calcPower, -calcPower, calcPower, calcPower);
                telemetry.addData("targetAngle", targetAngle);
                telemetry.update();
            }
        }
        power(0,0,0,0);
         */
    }
    public void miscare_fata(double targetY){
        double tY = targetY;
        double powerY;
        while(currentY <= tY){
            powerY = kp2*(targetY-currentY);
            if(powerY>1)powerY = 1;
            if(powerY < -1)powerY = -1;
            power(powerY,powerY,powerY,powerY);
        }
        power(0,0,0,0);

    }


    @Override
    public void runOpMode(){

        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motordf;
        encoderSpate = motorsf;
        encoderStanga = motorss;

        motordf.setPower(0);
        motords.setPower(0);
        motorsf.setPower(0);
        motorss.setPower(0);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motords.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorsf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motordf.setDirection(DcMotorSimple.Direction.REVERSE);


        stoneDetectorModified.stonesToFind = 1;
        stoneDetectorModified.useDefaults();
        stoneDetectorModified.filter = new SkystoneDetector(p1, p2);
        stoneDetectorModified.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetectorModified);
        webcam.startStreaming(resWidth, resHeight, OpenCvCameraRotation.SIDEWAYS_LEFT);
        Loc.start();

        //power(-0.3,-0.3,0.3,0.3); // se roteste cu minus

        while(!isStarted()) {
            if (stoneDetectorModified.foundRectangles().get(0).y > 406) {
                telemetry.addData("Skystone Position:", "LEFT");
            } else if (stoneDetectorModified.foundRectangles().get(0).y > 253) {
                telemetry.addData("Skystone Position", "CENTER");
            } else {
                telemetry.addData("Skystone Position", "RIGHT");
            }
            telemetry.addData("EncDr", EncDr);
            telemetry.addData("EncSt", EncSt);
            telemetry.addData("EncSp", EncSp);
            telemetry.addData("X", currentX);
            telemetry.addData("Y", currentY);
            telemetry.addData("atan", arctg);
            telemetry.addData("Angle ", angle);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("targetDistance", targetDist);
            telemetry.update();
        }
        waitForStart();
      //  calcule(10000,10000);
        miscare_fata(100);
    }
    private void power(double ds, double df, double ss, double sf){
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
