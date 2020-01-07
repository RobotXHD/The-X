package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDControllerTestConfig {
    public static double p = 0.08, i = 0 ,d = 0.45, setpoint = 0;
    public static double py = 0.0005, iy = 0, dy = 0.004, setpointY = 0;
    public static double px = 0.001, ix = 0, dx = 0.006, setpointX = 0;
    public static double sidewaysCalib = 45.8636, rotationCalib = 76.1;
    public static double toleranceX = 150, toleranceY = 150, toleranceRotatie = 2;
    public static int targetVerifications = 8;
}
