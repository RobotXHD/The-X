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

import static java.lang.Math.abs;

@TeleOp
public class TeleOp_Colect extends OpMode {
    /**declare the motors*/
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotor scissorDreapta;
    private DcMotor scissorStanga;
    private DcMotor  motorColectSt, motorColectDr;
    private ServoImplEx vexDr,vexSt;
    private Servo servoclamp, servoBrSt, servoBrDr;
    /**variable for changing the movement speed of the robot*/
    private int v = 2;
    /**variables for calculating the power for motors*/
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    /**variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop */
    private double forward, right, clockwise;
    /**variable that stops the threads when programs stop*/
    private boolean stop;
    /** variables that  toggle motors collect */
    private boolean apoz = false, alast = true, apoz2 = false, alast2 = true, apoz3 = false, alast3 = true;
    private double powerColect = 0.6, powerSlider;
    private TouchSensor endStopFata;

    private Thread Colect = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                if (gamepad2.dpad_up) {
                    scissorDreapta.setPower(0.6);
                    scissorStanga.setPower(0.6);
                } else if (gamepad2.dpad_down) {
                    scissorDreapta.setPower(-1);
                    scissorStanga.setPower(-1);
                } else if (gamepad2.left_bumper) {
                    scissorDreapta.setPower(-0.2);
                    scissorStanga.setPower(-0.2);
                } else {
                    scissorDreapta.setPower(0);
                    scissorStanga.setPower(0);
                }

                powerSlider = gamepad2.left_stick_y;
                if (powerSlider < 0 && !endStopFata.isPressed()){
                    vexDr.setPosition(0.5 +powerSlider/2);
                    vexSt.setPosition(0.5 - powerSlider/2);
                }
                else if (powerSlider > 0){
                    vexDr.setPosition(0.5 +powerSlider/2);
                    vexSt.setPosition(0.5 - powerSlider/2);
                }
                else {
                    vexDr.setPosition(0.5);
                    vexSt.setPosition(0.5);
                }

                /**set the collector motors on or off using the toggle*/
                boolean abut = gamepad2.b;
                if(alast != abut){
                    if(gamepad2.b) {
                        apoz = !apoz;
                        if (apoz){
                            motorColectSt.setPower(-powerColect);
                            //motorColectDr.setPower(powerColect);
                        }
                        else{
                            motorColectSt.setPower(0);
                          //  motorColectDr.setPower(0);
                        }
                    }
                    alast=abut;
                }

                boolean abut2 = gamepad2.x;
                if(alast2 != abut2){
                    if(gamepad2.x) {
                        apoz2 = !apoz2;
                        if (apoz2){
                            motorColectSt.setPower(powerColect);
                            //motorColectDr.setPower(-powerColect);
                        }
                        else{
                            motorColectSt.setPower(0);
                           // motorColectDr.setPower(0);
                        }
                    }
                    alast2=abut2;
                }

                boolean abut3 = gamepad2.y;
                if(alast3 != abut3){
                    if(gamepad2.y) {
                        apoz3 = !apoz3;
                        if (apoz3){
                           servoclamp.setPosition(configs.pozitie_servoClamp_maxim);
                        }
                        else{
                            servoclamp.setPosition(configs.pozitie_servoClamp_minim);
                        }
                    }
                    alast3=abut3;
                }
                
                if(gamepad1.dpad_down){
                    servoBrDr.setPosition(0);
                    servoBrSt.setPosition(1);
                }
                else if(gamepad1.dpad_up){
                    servoBrDr.setPosition(1);
                    servoBrSt.setPosition(0);
                }

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
                forward = gamepad1.left_stick_y;
                right = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;


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

    @Override
    public void init(){
        /**initialization motors*/
        motordf = hardwareMap.get(DcMotorEx.class, configs.dfName);
        motords = hardwareMap.get(DcMotorEx.class, configs.dsName);//encSt
        motorsf = hardwareMap.get(DcMotorEx.class, configs.sfName);//encDr
        motorss = hardwareMap.get(DcMotorEx.class, configs.ssName);//encSp

        motorColectDr = hardwareMap.get(DcMotor.class, configs.colectDrName);
        motorColectSt = hardwareMap.get(DcMotor.class, configs.colectStName);

        scissorDreapta = hardwareMap.dcMotor.get(configs.scissorDrName);
        scissorStanga = hardwareMap.dcMotor.get(configs.scissorStName);

        servoclamp = hardwareMap.servo.get("clamp");
        servoBrDr = hardwareMap.servo.get("brDr");
        servoBrSt = hardwareMap.servo.get("brSt");
        vexDr = hardwareMap.get(ServoImplEx.class, "vexDr");
        vexSt = hardwareMap.get(ServoImplEx.class, "vexSt");

        endStopFata = hardwareMap.touchSensor.get("endFata");

        vexDr.setPwmRange(new PwmControl.PwmRange(1000,2000));
        vexSt.setPwmRange(new PwmControl.PwmRange(1000,2000));

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        scissorStanga.setDirection(DcMotorSimple.Direction.REVERSE);


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


        /**start the thread*/
        Colect.start();
        Chassis.start();

    }

    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop(){

    }

    /**using the stop function to stop the threads */
    public void stop(){stop = true;}

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }

}
