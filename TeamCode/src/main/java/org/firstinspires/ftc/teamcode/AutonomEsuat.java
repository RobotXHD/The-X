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
import static java.lang.Math.abs;

@Autonomous
public class AutonomEsuat extends LinearOpMode {
    private OpenCvCamera webcam;
    private int resWidth = 800, resHeight = 448;
    private Point p1 = new Point(resHeight * 0.55, 0);
    private Point p2 = new Point(resHeight, resWidth);
    private StoneDetectorModified stoneDetectorModified = new StoneDetectorModified(p1, p2);
    private RevBulkData bulkData;
    private ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    private ExpansionHubEx expansionHub;
    private double forward, right, clockwise, tempforward, tempright;
    double lastCorrectedY = 0, lastCorrectedX = 0, correctedY = 0, correctedX = 0, deltaY, deltaX, sindeltay, sindeltax, cosdeltay, cosdeltax;
    double currentY, currentX, encRot;
    volatile double robotHeading, lastRobotHeading; // sens trigonometric +
    double d = 377, omniLengthMm = 188.49555921538759430775860299677, mmPerTick = omniLengthMm / 4000, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLengthMm / 360) * 4000;
    long EncSp, EncSt, EncDr, lastEncDr;
    boolean stop = false;
    volatile boolean encodereCitite = false;
    long timestamp, lastTimestamp, LAST;
    private ExpansionHubMotor motorss, motorsf, motords, motordf;
    double targetAngle = 0, targetDist, calculatedAngle, calcPower;
    double kp = 6, ki = 0, kd = 27, kp2 = 0.0015;
    int direction;
    boolean jumpDetected = false;
    boolean jumpNeeded = false;
    private double timeChange, lastTime, lastDelta, deltaSum, dDelta;

    private Thread Loc = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;

        @Override
        public void run() {
            while (!isStopRequested()) {
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorCurrentPosition(encoderSpate);
                encDrVal = bulkData.getMotorCurrentPosition(encoderDreapta) + 10;
                encStVal = bulkData.getMotorCurrentPosition(encoderStanga) + 10;
                EncSp = -encSpVal;
                EncSt = Math.round(-encStVal * 1.05);
                EncDr = -encDrVal;

                encRot = ((EncDr - EncSt) / 2.0);
                robotHeading = Math.toRadians(encRot / tickPerDeg) % (2 * PI);
                if(robotHeading < 0){
                    robotHeading = 2 * PI + robotHeading;
                }
                if(robotHeading == 0){
                    robotHeading = 2 * PI;
                }
                lastCorrectedY = correctedY;
                lastCorrectedX = correctedX;

                correctedY = (EncSt + EncDr) / 2.0;
                correctedX = EncSp + 0.8435451847031402522029576407934 * encRot;

                deltaY = correctedY - lastCorrectedY;
                deltaX = correctedX - lastCorrectedX;

                cosdeltay = (Math.cos(robotHeading) * deltaY);
                cosdeltax = (Math.cos((PI / 2) - robotHeading) * deltaX);
                sindeltay = -(Math.sin(robotHeading) * deltaY);
                sindeltax = (Math.sin((PI / 2) - robotHeading) * deltaX);

                currentY = currentY + (cosdeltay + cosdeltax) * mmPerTick;  //TODO: De verificat daca merg OK
                currentX = currentX + (sindeltay + sindeltax) * mmPerTick;
            }
        }
    });

    public void rotatie(double X, double Y) {
        jumpDetected = false;
        jumpNeeded = false;
        double dx = X - currentX;
        double dy = Y - currentY;
        double lastAngle, llastangle;

        if (dx > 0) {
            if (dy > 0) {
                //1
                calculatedAngle = 2*PI - (Math.atan(dx / dy));
            } else {
                //2
                calculatedAngle = 2*PI - (Math.atan(dx / dy) + PI);
            }
        } else {
            if (dy > 0) {
                //4
                calculatedAngle = 2*PI - (Math.atan(dx / dy) + 2 * PI);
            } else {
                //3
                calculatedAngle = 2*PI - (Math.atan(dx / dy) + PI);
            }
        }

        targetAngle = calculatedAngle - robotHeading;

        if(targetAngle < -PI){
            targetAngle = 2 * PI + targetAngle;
        }

        if(targetAngle < 0){
            direction = -1;
        }
        else{
            direction = 1;
        }
        
        //targetDist = Math.sqrt((X - currentX) * (X - currentX) + (Y - currentY) * (Y - currentY));

        lastAngle = targetAngle;
        llastangle = targetAngle;
        telemetry.addData("direction", direction);
        telemetry.addData("target", targetAngle);
        telemetry.update();
        sleep(1000);
        if(direction == 1){
            while((targetAngle < -0.0006 || targetAngle > 0.0006) && (lastAngle < -0.0006 || lastAngle > 0.0006)){
                telemetry.addData("EncDr", EncDr);
                telemetry.addData("EncSt", EncSt);
                telemetry.addData("EncSp", EncSp);
                telemetry.addData("X", currentX);
                telemetry.addData("Y", currentY);
                telemetry.update();
                lastAngle = llastangle;
                llastangle = targetAngle;
                targetAngle = calculatedAngle - robotHeading;

                if(targetAngle < -PI){
                    targetAngle = 2 * PI + targetAngle;
                }
                calcPower = PID(targetAngle, kp, ki, kd);
                if(calcPower > 1){
                    calcPower = 1;
                }
                power(calcPower, calcPower, -calcPower, -calcPower);
            }
        }
        else{
            while((targetAngle < -0.0006 || targetAngle > 0.0006) && (lastAngle < -0.0006 || lastAngle > 0.0006)){
                telemetry.addData("EncDr", EncDr);
                telemetry.addData("EncSt", EncSt);
                telemetry.addData("EncSp", EncSp);
                telemetry.addData("X", currentX);
                telemetry.addData("Y", currentY);
                telemetry.update();
                lastAngle = llastangle;
                llastangle = targetAngle;
                targetAngle = calculatedAngle - robotHeading;

                if(targetAngle < -PI){
                    targetAngle = 2 * PI + targetAngle;
                }
                calcPower = -PID(-targetAngle, kp, ki, kd);
                if(calcPower < -1){
                    calcPower = -1;
                }
                power(calcPower, calcPower, -calcPower, -calcPower);

            }
        }
        power(0,0,0,0);
    }


    private void miscare(double X, double Y) {
        double powerCalc, current;
        if(X > Y){
            Y = X;
            current = currentX;
        }
        else{
            current = currentY;
        }
        if(current < Y){
            while(current < Y-50){
                powerCalc = (Y - current) * kp2;
                /*if(powerCalc > 1){
                    powerCalc = 1;
                }

                 */
                current = X > Y ? currentX:currentY;
                power(powerCalc,powerCalc,powerCalc,powerCalc);
                telemetry.addData("EncDr", EncDr);
                telemetry.addData("EncSt", EncSt);
                telemetry.addData("EncSp", EncSp);
                telemetry.addData("X", currentX);
                telemetry.addData("Y", currentY);
                telemetry.update();
            }
        }
        else{
            while(Y < current-50){
                powerCalc = (Y - current) * kp2;
                /*if(powerCalc > 1){
                    powerCalc = 1;
                }

                 */
                current = X > Y ? currentX:currentY;
                power(powerCalc,powerCalc,powerCalc,powerCalc);
                telemetry.addData("EncDr", EncDr);
                telemetry.addData("EncSt", EncSt);
                telemetry.addData("EncSp", EncSp);
                telemetry.addData("X", currentX);
                telemetry.addData("Y", currentY);
                telemetry.update();
            }
        }
        power(0, 0, 0, 0);
    }

    public void trueMiscare(double X, double Y, int colectare){
        rotatie(X, Y);
        if(colectare == 1){

        }
        else if(colectare == -1){

        }
        miscare(X, Y);
    }


    @Override
    public void runOpMode() {

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

        while (!isStarted()) {
            if (stoneDetectorModified.foundRectangles().get(0).y > 260) {
                telemetry.addData("Skystone Position:", "LEFT");
                telemetry.addData("Y", stoneDetectorModified.foundRectangles().get(0).y);
            } else if (stoneDetectorModified.foundRectangles().get(0).y > 93) {
                telemetry.addData("Skystone Position", "CENTER");
                telemetry.addData("Y", stoneDetectorModified.foundRectangles().get(0).y);
            } else {
                telemetry.addData("Skystone Position", "RIGHT");
                telemetry.addData("Y", stoneDetectorModified.foundRectangles().get(0).y);
            }
            telemetry.addData("EncDr", EncDr);
            telemetry.addData("EncSt", EncSt);
            telemetry.addData("EncSp", EncSp);
            telemetry.addData("X", currentX);
            telemetry.addData("Y", currentY);
            telemetry.addData("atan", calculatedAngle);
            telemetry.addData("Robot Heading ", robotHeading);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("Direction", direction);
            telemetry.addData("Jump needed?", jumpNeeded);
            telemetry.addData("targetDistance", targetDist);
            telemetry.update();
        }
        waitForStart();
        //rotatie(10000,0);
        trueMiscare(0,500, 0);
        trueMiscare(500,1000,1);
        trueMiscare(0,500,0);
        trueMiscare(-1500,500,0);
        //trueMiscare(,500,0);
        sleep(500);
    }

    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }

    public double PID(double delta, double kp, double ki, double kd){
        timeChange = System.currentTimeMillis() - lastTime;
        deltaSum += (delta * timeChange);
        dDelta = (delta - lastDelta);
        lastDelta = delta;
        lastTime = System.currentTimeMillis();
        return kp * delta + ki * deltaSum + kd * dDelta;
    }
}
