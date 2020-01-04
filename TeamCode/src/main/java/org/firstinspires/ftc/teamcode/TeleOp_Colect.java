package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.abs;

@TeleOp
public class TeleOp_Colect extends OpMode {
    /**
     * declare the motors
     */
    private ExpansionHubEx expansionHubEx;
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotorEx scissorDreapta;
    private DcMotorEx scissorStanga;
    private DcMotor motorColectSt, motorColectDr;
    private Servo servoclamp, servoPlatformaSt, servoPlatformaDr, servoCapstone,servoParcare;
    private ServoImplEx vexSt, vexDr;
    /**
     * variable for changing the movement speed of the robot
     */
    private int v = 2;
    /**
     * variables for calculating the power for motors
     */
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    double pos = 0.1, sysTime;
    private long encScissorDr, encScissorSt, offsetDr = 0, offsetSt = 0;
    /**
     * variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop
     */
    private double forward, right, clockwise;
    private boolean stop;
    private boolean apoz = false, alast = true, apoz2 = false, alast2 = true, apoz3 = false, alast3 = true, eStrans = false;
    private double powerColect = 1, powerSlider;
    private TouchSensor  touchScissorDr, touchScissorSt, touchGheara;

    private Thread Colect = new Thread(new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                scissorDreapta.setPower(-gamepad2.left_stick_y);
                scissorStanga.setPower(-gamepad2.left_stick_y);
                /**set the collector motors on or off using the toggle*/
                boolean abut = gamepad2.b;
                if (alast != abut) {
                    if (gamepad2.b) {
                        apoz = !apoz;
                        if (apoz && !touchGheara.isPressed()) {
                            motorColectSt.setPower(-powerColect);
                            motorColectDr.setPower(powerColect);
                        } else {
                            motorColectSt.setPower(0);
                            motorColectDr.setPower(0);
                        }
                    }
                    alast = abut;
                }

                boolean abut2 = gamepad2.x;
                if (alast2 != abut2) {
                    if (gamepad2.x) {
                        apoz2 = !apoz2;
                        if (apoz2 && !touchGheara.isPressed()) {
                            motorColectSt.setPower(powerColect);
                            motorColectDr.setPower(-powerColect);
                   //         servoclamp.setPosition(0.65);
                        } else {
                            motorColectSt.setPower(0);
                            motorColectDr.setPower(0);
                        }
                    }
                    alast2 = abut2;
                }
                if(touchGheara.isPressed() && motorColectDr.getPower() != 0){
                    servoclamp.setPosition(0);
                    motorColectSt.setPower(0);
                    motorColectDr.setPower(0);
                    apoz = true;
                }


                boolean abut3 = gamepad2.y;
                if (alast3 != abut3) {
                    if (gamepad2.y) {
                        apoz3 = !apoz3;
                        if (apoz3) {
                            servoclamp.setPosition(0);
                        } else {
                            servoclamp.setPosition(0.6);
                        }
                    }
                    alast3 = abut3;
                }

               powerSlider = gamepad2.right_stick_y;
                if (powerSlider < 0) {
                    vexDr.setPosition(0.5 + powerSlider / 2);
                    vexSt.setPosition(0.5 - powerSlider / 2);
                } else if (powerSlider > 0) {
                    vexDr.setPosition(0.5 + powerSlider / 2);
                    vexSt.setPosition(0.5 - powerSlider / 2);
                } else {
                    vexDr.setPosition(0.5);
                    vexSt.setPosition(0.5);
                }

                if (gamepad1.dpad_down) {
                    servoPlatformaDr.setPosition(0);
                    servoPlatformaSt.setPosition(1);
                } else if (gamepad1.dpad_up) {
                    servoPlatformaDr.setPosition(1);
                    servoPlatformaSt.setPosition(0.5);
                }

               servoCapstone.setPosition((gamepad1.right_trigger + gamepad2.right_trigger) / 2);

            }
        }
    });

    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                if (gamepad1.right_bumper) {
                    v = 1;
                } else if (gamepad1.left_bumper) {
                    v = 2;
                }
                /**getting the gamepad joystick values*/
                forward = -gamepad1.left_stick_y;
                right = gamepad1.left_stick_x;
                clockwise = -gamepad1.right_stick_x;


                /**calculating the power for motors */
                df = forward + clockwise - right;
                ss = forward - clockwise - right;
                sf = -forward + clockwise - right;
                ds = -forward - clockwise - right;

                /**normalising the power values*/
                max = abs(sf);
                if (abs(df) > max) {
                    max = abs(df);
                }
                if (abs(ss) > max) {
                    max = abs(ss);
                }
                if (abs(ds) > max) {
                    max = abs(ds);
                }
                if (max > 1) {
                    sf /= max;
                    df /= max;
                    ss /= max;
                    ds /= max;
                }
                /** setting the speed of the chassis*/
                if (v == 1) {
                    POWER(df / 2.5, sf / 2.5, ds / 2.5, ss / 2.5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }
            }
        }
    });

    public Thread automation = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (touchGheara.isPressed()) {
                    apoz3 = false;
                    servoclamp.setPosition(1);
                    motorColectDr.setPower(0);
                    motorColectSt.setPower(0);
                }
            }
        }
    });

    private Thread scissorEncoders = new Thread(new Runnable() {
        long encoderScissorStanga, encoderScissorDreapta;

        @Override
        public void run() {
            while (!stop) {
                encoderScissorDreapta = scissorDreapta.getCurrentPosition();
                encoderScissorStanga = scissorStanga.getCurrentPosition();
                encScissorDr = encoderScissorDreapta - offsetDr;
                encScissorSt = encoderScissorStanga - offsetSt;
            }
        }
    });

    @Override
    public void init() {
        /**initialization motors*/
        motordf = hardwareMap.get(DcMotorEx.class, configs.dfName);
        motords = hardwareMap.get(DcMotorEx.class, configs.dsName);//encSt
        motorsf = hardwareMap.get(DcMotorEx.class, configs.sfName);//encDr
        motorss = hardwareMap.get(DcMotorEx.class, configs.ssName);//encSp

        motorColectDr = hardwareMap.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hardwareMap.get(DcMotor.class, configs.colectStName);

        //expansionHubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub Sisteme");
        scissorDreapta = hardwareMap.get(DcMotorEx.class,configs.scissorDrName);
        scissorStanga = hardwareMap.get(DcMotorEx.class,configs.scissorStName);

        servoclamp = hardwareMap.servo.get("clamp");
        servoPlatformaDr = hardwareMap.servo.get(configs.servoPlatformaDrName);
        servoPlatformaSt = hardwareMap.servo.get(configs.servoPlatformaStName);
        servoCapstone = hardwareMap.servo.get("capstone");
        servoParcare = hardwareMap.servo.get("parcare");

        vexDr = hardwareMap.get(ServoImplEx.class, "vexDr");
        vexSt = hardwareMap.get(ServoImplEx.class, "vexSt");


        touchGheara= hardwareMap.touchSensor.get(configs.touchGhearaName);

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);


        vexDr.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        vexSt.setPwmRange(new PwmControl.PwmRange(1000, 2000));

        /**set the mode of the motors*/
        scissorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scissorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sysTime= System.currentTimeMillis();
       // servoclamp.setPosition(1);
        servoCapstone.setPosition(0);
        /**start the thread*/
        Colect.start();
        Chassis.start();
        //automation.start();
        //scissorEncoders.start();
    }

    /**using the loop function to send the telemetry to the phone */
    @Override
    public void loop() {
        telemetry.addData("POZ", pos);
        telemetry.update();
      /*  telemetry.addData("EncDr", encScissorDr);
        telemetry.addData("EncSt", encScissorSt);
        telemetry.addData("Capstone", gamepad1.right_trigger);
        telemetry.update();*/
    }

    /**
     * using the stop function to stop the threads
     */
    public void stop() {
        stop = true;
    }

    /**
     *
     * the power function sets the motor's power
     */
    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }

}
