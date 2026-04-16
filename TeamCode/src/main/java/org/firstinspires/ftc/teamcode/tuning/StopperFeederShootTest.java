package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.DriveParams;


@TeleOp(name="TeleOp Feeder Shoot Test", group="Test")
public final class StopperFeederShootTest extends LinearOpMode implements DriveParams {

    private Servo stopper = null;
    private CRServo feeder = null;
    private DcMotorEx belt = null;
    private DcMotorEx shooterLeft = null;
    private DcMotorEx shooterRight = null;
    private DcMotorEx intake = null;
    private Servo shooterAngle = null;
    private double shooterSpeed = 0.64;

    // far side
    // 0.64 for battery power 12.10v

    @Override
    public void runOpMode()  {

        feeder = hardwareMap.get(CRServo.class, "feeder");
        stopper = hardwareMap.get(Servo.class, "stopper");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooter_left");
        shooterRight  = hardwareMap.get(DcMotorEx.class, "shooter_right");
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooterAngle = hardwareMap.get(Servo.class, "shooter_angle");
        shooterAngle.setPosition(SHOOTER_ANGLE_05);

        belt.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        stopper.setPosition(GATE_CLOSED_POSITION);
        //shooterLeft.setVelocity();
//        shooterLeft.setVelocityPIDFCoefficients(10, 0, 0, 13.5);
//        shooterRight.setVelocityPIDFCoefficients(10, 0, 0, 13.5);

        waitForStart();

        updateSubsystem();

        while (opModeIsActive()) {

            if (gamepad2.leftBumperWasPressed()) {
                shooterSpeed += 0.01;
                shooterLeft.setPower(shooterSpeed);
                shooterRight.setPower(shooterSpeed);
            } else if (gamepad2.rightBumperWasPressed()) {
                shooterSpeed -= 0.01;
                shooterLeft.setPower(shooterSpeed);
                shooterRight.setPower(shooterSpeed);
            }

            if (gamepad2.dpad_down) { //block the artifact
                feeder.setPower(FEEDER_STOP_POWER);
                stopper.setPosition(GATE_CLOSED_POSITION);

            } else if (gamepad2.dpad_up) {  //ready to shoot
                feeder.setPower(FEEDER_FEED_POWER);
                stopper.setPosition(GATE_OPEN_POSITION);

            }


            telemetry.addData("shooterLeft getPower)",  shooterLeft.getPower());
            telemetry.addData("shooterRight getPower)", shooterRight.getPower());
            telemetry.update();
        }
    }

    private void updateSubsystem() {
        belt.setPower(BELT_SPEED_CAP);
        intake.setPower(INTAKE_SPEED_CAP);
        shooterLeft.setPower(shooterSpeed);
        shooterRight.setPower(shooterSpeed);
    }
}
