package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

@TeleOp
@Disabled
public class TeleOP_S4T extends OpMode {
    /**declare the encooders*/
    private DcMotor encoderDreapta, encoderSpate, encoderStanga;
    private DcMotorEx motordf;
    /**declare the motors*/
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
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
    private double forward, rright, clockwise;
    /**variable that stops the threads when programs stop*/
    private boolean stop = false;
    /**variables that count the thread's fps*/
    private long fpsC=0 , fpsEncDr= 0 , fpsEncSp = 0 , fpsEncSt;
    private long fpsCLast , fpsEncSpLast , fpsEncDrLast, fpsEncStLast;
    /** variable that  holds the system current time milliseconds*/
    private long sysTimeC , sysTimeEncSp, sysTimeEncSt , sysTimeEncDr, sysTimeLoop,sysTime, nextLoopTime = 3000, nextLoop;
    /**variables that hold the encoders values*/
    public double EncDr , EncSp, EncSt;

    private Thread ENCDR = new Thread(new Runnable() {
        @Override
        public void run() {
            double encd;
            /**repeat until the program stops*/
            while(!stop){
                /**reading the encoders*/
                encd = encoderDreapta.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                EncDr = encd;

                /** fps counter */
                fpsEncDr++;
                if (sysTimeEncDr + 3000 < System.currentTimeMillis()) {
                    fpsEncDrLast = fpsEncDr / 3;
                    fpsEncDr = 0;
                    sysTimeEncDr = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCST = new Thread(new Runnable() {
        @Override
        public void run() {
            double enst;
            /**repeat until the program stops*/
            while(!stop){
                /**reading the encoders*/
                enst = encoderStanga.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                EncSt = enst;

                /** fps counter */
                fpsEncSt++;
                if (sysTimeEncSt + 3000 < System.currentTimeMillis()) {
                    fpsEncStLast = fpsEncSt / 3;
                    fpsEncSt = 0;
                    sysTimeEncSt= System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCSP = new Thread(new Runnable() {
        double encsp;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**reading the encoders*/
                encsp =  encoderSpate.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                EncSp =  encsp;

                /** fps counter */
                fpsEncSp++;
                if (sysTimeEncSp + 3000 < System.currentTimeMillis()) {
                    fpsEncSpLast = fpsEncSp/3;
                    fpsEncSp = 0;
                    sysTimeEncSp = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                /**change the variable that controls the speed of the chassis using the bumpers*/
                if (gamepad1.right_bumper) {
                    v = 1;
                } else if (gamepad1.left_bumper) {
                    v = 2;
                }
                /**getting the gamepad joystick values*/
                forward = gamepad1.left_stick_y;
                rright = -gamepad1.left_stick_x;
                clockwise = gamepad1.right_stick_x;

                /**calculating the power for motors */
                df = forward + clockwise - rright;
                ss = forward - clockwise - rright;
                sf = -forward + clockwise - rright;
                ds = -forward - clockwise - rright;

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
                /** fps counter */
                fpsC++;
                if (sysTimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    sysTimeC = System.currentTimeMillis();
                }

                /** setting the speed of the chassis*/
                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }
            }
        }
    });

    @Override
    public void init() {

        /**set the mode of the  motors and the encoders */
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");

        encoderDreapta = hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga= hardwareMap.get(DcMotor.class,"encoderStanga");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        /**set the mode of the  motors */
        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**initialization system current time milliseconds */
        sysTime = System.currentTimeMillis();
        sysTimeC = sysTime;
        sysTimeEncDr = sysTime;
        sysTimeEncSp = sysTime;
        sysTimeEncSt = sysTime;

        /**start the threads*/
        ENCDR.start();
        ENCSP.start();
        ENCST.start();
        Chassis.start();

    }

    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
            telemetry.addData("ENCDR:", EncDr);
            telemetry.addData("ENCSP:", EncSp);
             telemetry.addData("ENCST:", EncSt);
            telemetry.addData("motordf: ", motordf.getCurrentPosition());
            telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
            telemetry.addData("motords: ", motords.getCurrentPosition());
            telemetry.addData("motorss: ", motorss.getCurrentPosition());
            telemetry.addData("Th Chassis: ", fpsCLast);
            telemetry.addData("Th EncSP: ", fpsEncSpLast);
            telemetry.addData("Th EncDR: ", fpsEncDrLast);
            telemetry.addData("Th EncST: ", fpsEncStLast);
            telemetry.update();

    }
    /**using the stop function to stop the threads */
    public void stop(){stop = true;}

    /**the power functon sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }
}
