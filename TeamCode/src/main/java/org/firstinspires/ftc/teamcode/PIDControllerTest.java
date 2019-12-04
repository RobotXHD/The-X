package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class PIDControllerTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public double encDr, encSt, encSp;
    public RevBulkData bulkData;
    public ExpansionHubMotor encoderDreapta, encoderSpate, encoderStanga;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motorss, motorsf, motords, motordf;
    public double rotatie = 0, ticksPerDegree = 70.4383556555;
    public PIDControllerAdevarat pidRotatie = new PIDControllerAdevarat(0,0,0);
    public PIDControllerAdevarat pidY = new PIDControllerAdevarat(0,0,0);
    public double KPR = 0, KIR = 0, KDR = 0, SETPOINT = 0, correctionR;
    public double KPY = 0, KIY = 0, KDY = 0, SETPOINTY = 0, correctionY;
    public double ds, df, ss, sf, Y;

    @Override
    public void runOpMode() throws InterruptedException {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motorss = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.ssName);
        motords = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dsName);
        motorsf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.sfName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        encoderDreapta = motorss;
        encoderSpate = motorsf;
        encoderStanga = motordf;

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

        encoderRead.start();
        waitForStart();
        pidRotatie.setSetpoint(0);
        pidY.setSetpoint(0);
        pidRotatie.enable();
        pidY.enable();
        while(!isStopRequested()){
            if(KPR != PIDControllerTestConfig.p || KIR != PIDControllerTestConfig.i || KDR != PIDControllerTestConfig.d){
                pidRotatie.setPID(PIDControllerTestConfig.p, PIDControllerTestConfig.i, PIDControllerTestConfig.d);
                KPR = PIDControllerTestConfig.p;
                KIR = PIDControllerTestConfig.i;
                KDR = PIDControllerTestConfig.d;
            }
            if(SETPOINT != PIDControllerTestConfig.setpoint){
                pidRotatie.setSetpoint(PIDControllerTestConfig.setpoint);
                SETPOINT = PIDControllerTestConfig.setpoint;
            }
            if(KPY != PIDControllerTestConfig.py || KIY != PIDControllerTestConfig.iy || KDY != PIDControllerTestConfig.dy){
                pidY.setPID(PIDControllerTestConfig.py, PIDControllerTestConfig.iy, PIDControllerTestConfig.dy);
                KPY = PIDControllerTestConfig.py;
                KIY = PIDControllerTestConfig.iy;
                KDY = PIDControllerTestConfig.dy;
            }
            if(SETPOINTY != PIDControllerTestConfig.setpointY){
                pidY.setSetpoint(PIDControllerTestConfig.setpointY);
                SETPOINTY = PIDControllerTestConfig.setpointY;
            }
            Y = (encDr + encSt)/2;
            correctionR = pidRotatie.performPID(rotatie);
            correctionY = pidY.performPID(Y);
            ds = correctionR + correctionY;
            df = correctionR + correctionY;
            ss = -correctionR + correctionY;
            sf = -correctionR + correctionY;
            power(ds, df, ss, sf);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("P", pidY.getP() * pidY.getError());
            packet.put("I", pidY.getI() * pidY.getISum());
            packet.put("D", pidY.getD() * pidY.getDError());
            packet.put("PIDR", correctionR);
            packet.put("PIDY", correctionY);
            packet.put("ErrorY", pidY.getError());
            packet.put("Y", Y);
            dashboard.sendTelemetryPacket(packet);
        }
    }
    private Thread encoderRead = new Thread(new Runnable() {
        long st, dr, sp;
        @Override
        public void run() {
            while(!isStopRequested()){
                bulkData = expansionHub.getBulkInputData();
                st = bulkData.getMotorCurrentPosition(encoderStanga);
                sp = bulkData.getMotorCurrentPosition(encoderSpate);
                dr = bulkData.getMotorCurrentPosition(encoderDreapta);
                rotatie = ((dr - st)/2.0)/ticksPerDegree;
                encDr = dr + rotatie * ticksPerDegree;
                encSp = sp;
                encSt = st - rotatie * ticksPerDegree;
            }
        }
    });
    private void power(double ds, double df, double ss, double sf) {
        motordf.setPower(df);
        motorss.setPower(ss);
        motorsf.setPower(sf);
        motords.setPower(ds);
    }
}
