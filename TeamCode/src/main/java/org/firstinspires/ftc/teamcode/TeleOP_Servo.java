package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TeleOP_Servo extends OpMode {
    public Servo  servoPlatformaSt, servoPlatformaDr;
    public double pozStsus = 1, pozStjos = 0.1, pozDrsus = 1, pozDrjos = 0, systime;
    public boolean stop = false;

    @Override
    public void init() {
        servoPlatformaDr = hardwareMap.servo.get(configs.servoPlatformaDrName);
        servoPlatformaSt = hardwareMap.servo.get(configs.servoPlatformaStName);

        systime = System.currentTimeMillis();
        Servo.start();
    }

    public Thread Servo = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (systime + 200 < System.currentTimeMillis()) {

                    if (gamepad1.dpad_left && pozStsus < 1) {
                        pozStsus += 0.01;
                        servoPlatformaSt.setPosition(pozStsus);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.dpad_right && pozStsus > 0.5) {
                        pozStsus -= 0.01;
                        servoPlatformaSt.setPosition(pozStsus);
                        systime = System.currentTimeMillis();
                    }

                    if (gamepad1.dpad_up && pozStjos < 0.5) {
                        pozStjos += 0.01;
                        servoPlatformaSt.setPosition(pozStjos);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.dpad_down && pozStjos > 0) {
                        pozStjos -= 0.01;
                        servoPlatformaSt.setPosition(pozStjos);
                        systime = System.currentTimeMillis();
                    }

                    if(gamepad1.a && pozDrsus < 1){
                        pozDrsus += 0.01;
                        servoPlatformaDr.setPosition(pozDrsus);
                        systime = System.currentTimeMillis();
                    }
                    else if(gamepad1.b && pozDrsus > 0.5){
                        pozDrsus -= 0.01;
                        servoPlatformaDr.setPosition(pozDrsus);
                        systime = System.currentTimeMillis();
                    }

                    if(gamepad1.x && pozDrjos < 0.5){
                        pozDrjos += 0.01;
                        servoPlatformaDr.setPosition(pozDrjos);
                        systime = System.currentTimeMillis();
                    }
                    else if(gamepad1.y && pozDrjos > 0){
                        pozDrjos -= 0.01;
                        servoPlatformaDr.setPosition(pozDrjos);
                        systime = System.currentTimeMillis();
                    }
                }

            }
        }
    });

    @Override
    public void loop() {
        telemetry.addData("dr: ", servoPlatformaDr.getPosition());
        telemetry.addData("st: ", servoPlatformaSt.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
    }
}
