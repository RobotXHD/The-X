package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


@TeleOp

public class moto_moto_motorola extends OpMode {
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motordf;
    public PIDControllerAdevarat pid = new PIDControllerAdevarat(0,0,0);
    public double correction;
    public double df, encdf;
    public boolean stop = false;
    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);

        motordf.setVelocity(0);
        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pid.setOutputRange(30000.0, -30000.0);
        pid.setTolerance(PIDControllerTestConfigTest.tolerance);
        pid.setPID(PIDControllerTestConfigTest.p, PIDControllerTestConfigTest.i, PIDControllerTestConfigTest.d);
        pid.setSetpoint(PIDControllerTestConfigTest.setpoint);
        pid.enable();

    }

    @Override
    public void loop(){
        bulkData = expansionHub.getBulkInputData();
        encdf = bulkData.getMotorCurrentPosition(motordf);
        correction = pid.performPID(encdf);
        motordf.setVelocity(correction*30000);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("EncDr: ", df - PIDControllerTestConfigTest.setpoint);
        telemetryPacket.put("P", pid.getP() * pid.getError());
        telemetryPacket.put("I", pid.getI() * pid.getISum());
        telemetryPacket.put("D", pid.getD() * pid.getDError());
        telemetryPacket.put("PID", correction);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public void stop() {
        stop = true;
    }
}
