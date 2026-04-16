package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    public Shooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter_left");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooter_right");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

    }

    public void run(double speed) {
        shooterLeft.setPower(speed);
        shooterRight.setPower(speed);
    }

    public void stop() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

}