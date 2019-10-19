package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class Constants {

    //----------------------------------------------------------------------------------------------
    // Drive Constants
    //----------------------------------------------------------------------------------------------

    public static double  DRIVE_SPEED = 0.7,
                                STRAFE_SPEED = 0.7,
                                ROTATE_SPEED = 0.4,
                                ARM_ROTATE_SPEED = 1.0,
                                ARM_EXTEND_POWER = 0.8,
                                WHEEL_CIRCUMFERENCE = 2 * Math.PI * 1.0,
                                TICKS_PER_REVOLUTION =  MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class).getTicksPerRev(),
                                GEAR_RATIO = 0.4, // 2:5
                                ROBOT_WIDTH = 14.0,
                                DRIVE_SMOOTHING_FACTOR = 0.97,
                                STRAFE_SMOOTHING_FACTOR = 0.97;

    public static PIDCoefficients DRIVE_PID_COEFFICIENTS = new PIDCoefficients(0.0, 0.0, 0.0),
                                        STRAFE_PID_COEFFICIENTS = new PIDCoefficients(0.0, 0.0, 0.0),
                                        ROTATE_PID_COEFFICIENTS = new PIDCoefficients(0.0, 0.0, 0.0);

    //----------------------------------------------------------------------------------------------
    // Vision Constants
    //----------------------------------------------------------------------------------------------

    public static String  VUFORIA_LICENSE_KEY = "AQOGHXH/////AAABme0x8ObyAUzii3UV4T02I5lw0gxq8oJY69zk3Asw0BUT+3jrPNRQxi4JmOTUmEnIT4f536LplHYTEiTsB9RU4RE7KqhhyQcQWo9oJZLyFU7m1xoMB/dhbyTJ7i0RZcyqsK1QmYd3Ihu0XdJBG58YBrDctb5aJi+zG2tgezJo2zceT7sNMI15rk2uV+vl3C3RWxFwJOH9SiSIE5cKdOqUmOMGNjbULVhU8IeI+EWt+RqX9jmiHXRoe7o2iqODLTUWKb8Btn6O9tPQfxT4FwALD6Ss1wHTySzs5A3j0Y4t7K6NlbW+74UUI/gQmsTnnza/4fNMTTAaquanDHRYbuc6e+zvXAOMa9J6Y67jDSdpWv95";

    public static float   MM_PER_INCH = 25.4f,
                                STONE_Z = 2.00f * MM_PER_INCH;

    public static float   CAMERA_FORWARD_DISPLACEMENT  = 4.0f * MM_PER_INCH,   // eg: Camera is 4 Inches in front of robot-center
                                CAMERA_VERTICAL_DISPLACEMENT = 8.0f * MM_PER_INCH,   // eg: Camera is 8 Inches above ground
                                CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    public static double  SKYSTONE_LEFT_BOUND = -10,
                                SKYSTONE_RIGHT_BOUND = 10;
}
