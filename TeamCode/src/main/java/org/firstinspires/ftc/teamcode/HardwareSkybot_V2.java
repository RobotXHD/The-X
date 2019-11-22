package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;

public class HardwareSkybot_V2 extends LinearOpMode {
    public boolean startTh;
    public double encDr, encSt, encSp;
    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf;
    public double encoderDrTotal = 0, encoderStTotal = 0, encoderSpTotal = 0;
    public double rotatie = 0, rotatieTotala = 0, delta = 0;
    public double kp = 0.0001;
    public double wheelDiameter = 60;
    public double powerDr, powerSt;
    public long calculatedTicks = 0;
    public HardwareSkybot_V2(boolean startThreads){
        startTh = startThreads;
    }

    public void Init(HardwareMap hard){
        expansionHub = hard.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hard.get(DcMotorEx.class, configs.dfName);

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

        if(startTh){
            encoderReading.start();
        }
    }

    public Thread encoderReading = new Thread(new Runnable() {
        long encSpVal, encDrVal, encStVal;
        @Override
        public void run() {
            while (startTh){
                bulkData = expansionHub.getBulkInputData();
                encSpVal = bulkData.getMotorCurrentPosition(encoderSpate);
                encDrVal = bulkData.getMotorCurrentPosition(encoderDreapta);
                encStVal = bulkData.getMotorCurrentPosition(encoderStanga);
                encSp = -encSpVal;
                encSt = -encStVal;
                encDr = -encDrVal;
                rotatie = (encDr - encSt)/2;
            }
        }
    });

    public void fata(double distanta, double putere){
        calculatedTicks = Math.round(((distanta - 100 )* 4000)/(PI * wheelDiameter));
        encoderDrTotal += calculatedTicks;
        encoderStTotal += calculatedTicks;
        rotatieTotala = rotatie;

        while(encDr < encoderDrTotal && encSt < encoderStTotal){

            delta = rotatieTotala - rotatie;
            powerDr = putere + delta * kp;
            if(powerDr > putere){
                powerDr = putere;
            }
            powerSt = putere - delta * kp;
            if(powerSt > putere){
                powerSt = putere;
            }

            power(powerDr , powerDr, powerSt, powerSt);
        }
        power(0,0,0,0);
    }

    public void stanga(double distanta, double putere){
        calculatedTicks = -Math.round(((distanta - 100)* 4000)/(PI * wheelDiameter));
        encoderSpTotal += calculatedTicks;
        rotatieTotala = rotatie;

        while(encSp > encoderSpTotal){

            delta = rotatieTotala - rotatie;
            powerDr = putere + delta * kp;
            if(powerDr > putere){
                powerDr = putere;
            }
            powerSt = putere - delta * kp;
            if(powerSt > putere){
                powerSt = putere;
            }

            power(-powerDr, powerDr, powerSt, -powerSt);
        }
        power(0,0,0,0);
    }



    @Override
    public void runOpMode() {

    }

    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
