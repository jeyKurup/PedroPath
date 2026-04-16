package org.firstinspires.ftc.teamcode.common;

public interface DriveParams {

    double STOP_POWER = 0;
    // ------------ INTAKE CONSTANTS ------------
    double INTAKE_80P_POWER = 0.8;
    double INTAKE_90P_POWER = 0.9;
    double INTAKE_SPEED_CAP = 0.75;
    double INTAKE_FULL_POWER = 1.0;

    // ------------ SHOOTER CONSTANTS ------------
    double SHOOTER_FULL_POWER = 1.0;
    double SHOOTER_50P_POWER = 0.5;
    double SHOOTER_53P_POWER = 0.53;
    double SHOOTER_52P_POWER = 0.52;
    double SHOOTER_55P_POWER = 0.55;
    double SHOOTER_56P_POWER = 0.57;
    double SHOOTER_57P_POWER = 0.57;
    double SHOOTER_60P_POWER = 0.60;
    double SHOOTER_62P_POWER = 0.62;
    double SHOOTER_65P_POWER = 0.65;
    double SHOOTER_70P_POWER = 0.70;
    // 5000 RPM goBILDA bare motor: 28 ticks/rev -> max ~2333 ticks/sec; 0.70 * 2333 ≈ 1633
    double SHOOTER_70P_VELOCITY = 1633.0;
    double SHOOTER_72P_POWER = 0.72;
    double SHOOTER_75P_POWER = 0.75;
    double SHOOTER_80P_POWER = 0.8;
    double SHOOTER_90P_POWER = 0.9;

    // ------------ BELT CONSTANTS ------------
    double BELT_FULL_POWER = 1.0;
    double BELT_80P_POWER = 0.8;
    double BELT_SPEED_CAP   = 0.80;

    // ------------ GATE CONSTANTS ------------
    double GATE_OPEN_POSITION = 0.75;
    double GATE_CLOSED_POSITION = 0.55;
    double GATE_OPEN_TIME = 2.5;

    // ------------ MOTIF CONSTANTS ------------
    int MOTIF_LEFT = 21;
    int MOTIF_CENTER = 22;
    double SPEED_CAP_MIN    = 0.7;
    double SPEED_CAP_MAX    = 0.90;


    double SPEED_CAP_SHOOTER = 0.80;

    //    double ALIGNMENT_THRESHOLD_X = 1.0; // degrees
    double ALIGNMENT_THRESHOLD_Y = 1.0; // degrees
    double MAX_STRAFE_POWER = 0.3;
    double MAX_FORWARD_POWER = 0.3;
//    double MAX_TURN_POWER = 0.50;

    // PID-like constants for smoother alignment
    double KP_STRAFE = 0.02;
    double KP_FORWARD = 0.02;
//    double KP_TURN = 0.015;

    double SHOOTER_ANGLE_05  = 0.05;

    double KP_TURN = 0.05; //0.04; prev //value    // Proportional gain for turning (start here, tune as needed)
    double MAX_TURN_POWER = 0.35;       // Maximum turn speed (adjust for your robot)
    double ALIGNMENT_THRESHOLD_X = 0.1; // Degrees - how close to center is "aligned"
    double BLUE_IND = 0.611;
    double RED_IND = 0.288;
    double YELLOW_IND =  0.380;

    double NOMINAL_VOLTAGE = 12.0;

    public static double MAX_COMPENSATION = 1.15;  // Don't exceed 115% power
    public static double MIN_VOLTAGE = 10.0;       // Minimum safe voltage
    double FEEDER_STOP_POWER = 0.0;
    double FEEDER_FEED_POWER = 1.0;
    double READY_STATE_WAIT_TIME = 1.0;

}