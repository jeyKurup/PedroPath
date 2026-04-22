package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.DriveParams;

public class Shooter implements DriveParams {
    private DcMotorEx shooter;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotor.Direction.REVERSE);


    }

    public void run(double speed) {
        shooter.setPower(speed);
    }

    public void stop() {
        shooter.setPower(0);
    }

    public double getShooterSpeed(boolean insideFlag, double voltage) {
        double speed = 0.0;
        if (insideFlag) {
            // Inside shooting
            speed = SHOOTER_55P_POWER;
            if (voltage > 13.8) {
                speed = SHOOTER_53P_POWER;
            }else if (voltage > 13.0){
                speed = SHOOTER_55P_POWER;
            }
        }else {
            // Outside/farside shooting
            speed = SHOOTER_67P_POWER;
            if (voltage > 13.8) {   //this is good - tested
                speed = SHOOTER_60P_POWER;
            } else if (voltage > 13.3) {
                    speed = SHOOTER_62P_POWER;
            }else if (voltage > 13.0){
                speed = 0.65;
            }
        }
        return speed;
    }


}