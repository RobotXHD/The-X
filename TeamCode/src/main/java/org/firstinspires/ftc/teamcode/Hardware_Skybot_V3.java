package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Hardware_Skybot_V3 extends LinearOpMode {
    public boolean startTh = false;
    public double encDr, encSt, encSp;
    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf;
    public double Rotatie = 0, ticksPerDegree = PIDControllerTestConfig.rotationCalib;
    public PIDControllerAdevarat pidRotatie = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidY = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidX = new PIDControllerAdevarat(0,0,0);
    public double correctionR,correctionY,correctionX;
    public double ds, df, ss, sf, Y, tempRot, max;
    public int verifications = 0;

    public Hardware_Skybot_V3(boolean startThreads) {startTh = startThreads;}
    public Hardware_Skybot_V3(){}

    public  void Init(HardwareMap hard){
        expansionHub = hard.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motorss;
        encoderSpate = motordf;
        encoderStanga = motorsf;

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

        if(startTh){
            encoderRead.start();
        }
        pidRotatie.setSetpoint(0);
        pidY.setSetpoint(0);
        pidX.setSetpoint(0);

        pidRotatie.setTolerance(PIDControllerTestConfig.toleranceRotatie);
        pidY.setTolerance(PIDControllerTestConfig.toleranceY);
        pidX.setTolerance(PIDControllerTestConfig.toleranceX);

        pidRotatie.setPID(PIDControllerTestConfig.p, PIDControllerTestConfig.i, PIDControllerTestConfig.d);
        pidY.setPID(PIDControllerTestConfig.py, PIDControllerTestConfig.iy, PIDControllerTestConfig.dy);
        pidX.setPID(PIDControllerTestConfig.px, PIDControllerTestConfig.ix, PIDControllerTestConfig.dx);

        pidRotatie.enable();
        pidY.enable();
        pidX.enable();
    }

    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }

    @Override
    public void runOpMode() {
    }

    public void gotoY(double setPointY){
        verifications = 0;
        pidY.setSetpoint(-setPointY);
        do{
            Y = (encDr + encSt) / 2;
            correctionR = pidRotatie.performPID(Rotatie);
            correctionY = pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df) : max;
            max = Math.abs(sf) > max ? Math.abs(sf) : max;
            max = Math.abs(ss) > max ? Math.abs(ss) : max;

            if (max > 1) {
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            power(ds, df, ss, sf);
            verifications = pidY.onTarget() ? verifications+1 : 0;
        }while(verifications < PIDControllerTestConfig.targetVerifications);
        power(0,0,0,0);
    }

    public void rotatie(double setPoint){
        verifications = 0;
        pidRotatie.setSetpoint(setPoint);
        do{
            Y = (encDr + encSt)/2;
            correctionR = pidRotatie.performPID(Rotatie);
            correctionY = pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > 1){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            power(ds, df, ss, sf);
            verifications = pidRotatie.onTarget() ? verifications+1 : 0;
        }while (verifications < PIDControllerTestConfig.targetVerifications);
        power(0,0,0,0);
    }

    public void gotoX(double setPointX){
        verifications = 0;
        pidX.setSetpoint(setPointX);
        do{
            Y = (encDr + encSt)/2;
            correctionR = pidRotatie.performPID(Rotatie);
            correctionY = pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > 1){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            power(ds, df, ss, sf);
            verifications = pidX.onTarget() ? verifications+1 : 0;
        }while(verifications < PIDControllerTestConfig.targetVerifications);
        power(0,0,0,0);
    }




    private Thread encoderRead = new Thread(new Runnable() {
        long st, dr, sp;
        @Override
        public void run() {
            while (startTh) {
                bulkData = expansionHub.getBulkInputData();
                st = -bulkData.getMotorCurrentPosition(encoderStanga);
                sp = -bulkData.getMotorCurrentPosition(encoderSpate);
                dr = bulkData.getMotorCurrentPosition(encoderDreapta);
                tempRot = ((dr - st) / 2.0);
                Rotatie = tempRot / ticksPerDegree;
                encDr = dr + Rotatie * ticksPerDegree;
                encSp = sp + Rotatie * PIDControllerTestConfig.sidewaysCalib;
                encSt = st - Rotatie * ticksPerDegree;
            }
        }
    });

}
