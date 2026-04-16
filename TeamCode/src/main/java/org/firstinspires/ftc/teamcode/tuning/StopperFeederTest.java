package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.DriveParams;

@Disabled
@TeleOp(name="TeleOp Feeder Test", group="Test")
public final class StopperFeederTest extends LinearOpMode implements DriveParams {

    private Servo stopper = null;
    private CRServo feeder = null;

    @Override
    public void runOpMode() throws InterruptedException {

        feeder = hardwareMap.get(CRServo.class, "feeder");
        stopper = hardwareMap.get(Servo.class, "stopper");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.dpad_down) { //block the artifact
                feeder.setPower(FEEDER_STOP_POWER);
                //sleep(2);
                stopper.setPosition(GATE_CLOSED_POSITION);

            } else if (gamepad2.dpad_up) {  //ready to shoot
                feeder.setPower(FEEDER_FEED_POWER);
                stopper.setPosition(GATE_OPEN_POSITION);

            }
        }
    }
}
