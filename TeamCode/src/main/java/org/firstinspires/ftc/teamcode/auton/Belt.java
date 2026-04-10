package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.DriveParams;

public class Belt implements DriveParams {
    private DcMotorEx belt;

    public Belt(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setDirection(DcMotor.Direction.FORWARD);
    }

    public void run(double speed) {
         belt.setPower(speed);
    }

    public void run() {
        belt.setPower(BELT_80P_POWER);
    }


    public void stop() {
        belt.setPower(STOP_POWER);
    }

}