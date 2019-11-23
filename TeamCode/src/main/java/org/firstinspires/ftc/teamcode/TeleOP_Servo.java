package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Disabled
public class TeleOP_Servo extends OpMode {
    public Servo servorot, servosj, servoclamp;
    public ServoImplEx servomonster;
    public double pozsjsus = 1, pozsjjos = 0.1, pozrotsus = 1, pozrotjos = 0, systime, pozmonstersus = 1, pozmonsterjos = 0;
    public boolean stop = false;

    @Override
    public void init() {
        servorot = hardwareMap.get(Servo.class, "rot");
        servosj = hardwareMap.get(Servo.class, "sj");

        servomonster = hardwareMap.get(ServoImplEx.class, "monster");
        servomonster.setPwmRange(new PwmControl.PwmRange(750,2250));
        servoclamp = hardwareMap.get(Servo.class,"clamp");

        servosj.setPosition(0);
        servorot.setPosition(0);
        systime = System.currentTimeMillis();
        Servo.start();
    }

    public Thread Servo = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (systime + 200 < System.currentTimeMillis()) {

                    if (gamepad1.dpad_left && pozrotsus < 1) {
                        pozrotsus += 0.01;
                        servorot.setPosition(pozrotsus);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.dpad_right && pozrotsus > 0.5) {
                        pozrotsus -= 0.01;
                        servorot.setPosition(pozrotsus);
                        systime = System.currentTimeMillis();
                    }

                    if (gamepad1.dpad_up && pozrotjos < 0.5) {
                        pozrotjos += 0.01;
                        servorot.setPosition(pozrotjos);
                        systime = System.currentTimeMillis();
                    } else if (gamepad1.dpad_down && pozrotjos > 0) {
                        pozrotjos -= 0.01;
                        servorot.setPosition(pozrotjos);
                        systime = System.currentTimeMillis();
                    }

                    if(gamepad1.a && pozsjsus < 1){
                        pozsjsus += 0.01;
                        servosj.setPosition(pozsjsus);
                        systime = System.currentTimeMillis();
                    }
                    else if(gamepad1.b && pozsjsus > 0.5){
                        pozsjsus -= 0.01;
                        servosj.setPosition(pozsjsus);
                        systime = System.currentTimeMillis();
                    }

                    if(gamepad1.x && pozsjjos < 0.5){
                        pozsjjos += 0.01;
                        servosj.setPosition(pozsjjos);
                        systime = System.currentTimeMillis();
                    }
                    else if(gamepad1.y && pozsjjos > 0){
                        pozsjjos -= 0.01;
                        servosj.setPosition(pozsjjos);
                        systime = System.currentTimeMillis();
                    }
                }
                if(gamepad1.left_bumper){
                    servosj.setPosition(pozsjjos);
                    servorot.setPosition(pozrotjos);
                }
                else if(gamepad1.right_bumper){
                    servosj.setPosition(pozsjsus);
                    servorot.setPosition(pozrotsus);
                }
            }
        }
    });

    @Override
    public void loop() {
        telemetry.addData("Rotpoz: ", servorot.getPosition());
        telemetry.addData("Rotsj: ", servosj.getPosition());
        telemetry.addData("Rotmonster: ", servomonster.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        stop = true;
    }
}
