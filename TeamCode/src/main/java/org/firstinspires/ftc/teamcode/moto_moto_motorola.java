package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


@TeleOp

public class moto_moto_motorola extends OpMode {
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor motordf;
    public PIDControllerAdevarat pid = new PIDControllerAdevarat(0,0,0);
    public double correction;
    public double df, encdf;
    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, configs.expansionHubOdometrieName);
        motordf = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, configs.dfName);
        motordf.setVelocity(0);
        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motordf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        encdf = bulkData.getMotorCurrentPosition(motordf);
        pid.setTolerance(PIDControllerTestConfigTest.tolerance);
        pid.setPID(PIDControllerTestConfigTest.p, PIDControllerTestConfigTest.i, PIDControllerTestConfigTest.d);
        pid.setSetpoint(PIDControllerTestConfigTest.setpoint);

        correction = pid.performPID(encdf);
        motordf.setVelocity(correction);
    }
}
