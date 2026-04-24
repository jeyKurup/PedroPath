package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.DriveParams;

public class FeederFlywheelLogic implements DriveParams {

    private Servo gateServo;
    private CRServo feederServo;

    private Servo shooterAngleServo;


    private ElapsedTime stateTimer = new ElapsedTime();

    private boolean readyToFireShots = false;
    private enum FeederState {
        IDLE,
        OPEN_GATE_AND_SPIN_FEEDER,
        CLOSE_GATE_AND_STOP_FEEDER,

    }

    private FeederState feederState;

    public void init(HardwareMap hwMap) {
        gateServo = hwMap.get(Servo.class, "stopper");
        feederServo = hwMap.get(CRServo.class, "feeder");
        shooterAngleServo = hwMap.get(Servo.class, "shooter_angle");

        feederState = FeederState.IDLE;

        feederServo.setPower(0);
        gateServo.setPosition(GATE_CLOSED_POSITION);
        shooterAngleServo.setPosition(SHOOTER_ANGLE_05);
    }

    public void update() {
        switch (feederState) {
            case IDLE:
                if (readyToFireShots) {
                    gateServo.setPosition(GATE_CLOSED_POSITION);
                    // set velocity
                    feederServo.setPower(FEEDER_STOP_POWER);

                    stateTimer.reset();
                    feederState = FeederState.OPEN_GATE_AND_SPIN_FEEDER;
                }
                break;

            case OPEN_GATE_AND_SPIN_FEEDER:
                // set velocity
                gateServo.setPosition(GATE_OPEN_POSITION);
                feederServo.setPower(FEEDER_FEED_POWER);

                if (stateTimer.seconds() > GATE_OPEN_TIME) {
                    stateTimer.reset();
                    feederState = FeederState.CLOSE_GATE_AND_STOP_FEEDER;
                }
                break;

            case CLOSE_GATE_AND_STOP_FEEDER:

                readyToFireShots = false;
                gateServo.setPosition(GATE_CLOSED_POSITION);
                feederServo.setPower(FEEDER_STOP_POWER);

                if (stateTimer.seconds() > GATE_CLOSE_DELAY) {
                    stateTimer.reset();
                    feederState = FeederState.IDLE;
                }
                break;

        }
    }

    public void fireShots(boolean ready) {
        readyToFireShots = ready;
    }

    public boolean isReadyToFireShots() {
        return readyToFireShots;
    }
    public boolean isBusy() {
        return feederState != FeederState.IDLE;
    }
}