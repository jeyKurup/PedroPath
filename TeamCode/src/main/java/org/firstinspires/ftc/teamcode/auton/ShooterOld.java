package org.firstinspires.ftc.teamcode.auton;
/*
 DO NOT USE THIS CLASS ANYMORE. IT IS DEPRECATED.
 Use Shooter.java
*/

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Deprecated
public class ShooterOld {
    private DcMotorEx shooter;

    public ShooterOld(HardwareMap hardwareMap) {
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

}