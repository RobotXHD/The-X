package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp
public class Automatizari extends OpMode {
    AnalogInput potentiometru;
    @Override
    public void init() {
        potentiometru = hardwareMap.analogInput.get("pot");

    }


    @Override
    public void loop() {
        telemetry.addData("Pot", potentiometru.getVoltage());
        telemetry.update();
    }
}
