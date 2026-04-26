package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.DriveParams;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "BLUE Inside 2Gate+3Stack", group = "blue", preselectTeleOp = "TeleOp BLUE Decode Drive Game")
public class BlueInsideThreeStackWithTwoGateOpen extends OpMode implements DriveParams {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    // ---------- FEEDER FLYWHEEL-SHOOTER SETUP ----------
    private FeederFlywheelLogic feederStopper = new FeederFlywheelLogic();
    private Shooter shooter;
    private Intake intake;
    private Belt belt;
    private boolean shotsTriggered = false;
    private VoltageSensor voltageSensor;
    private double shooterSpeed = SHOOTER_55P_POWER;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_STACK1POS_GATE1POS,
        DRIVE_GATE1POS_SHOOTPOS,
        SHOOT_STACK1,
        DRIVE_STACK2POS_GATE2POS,
        DRIVE_GATE2POS_SHOOTPOS,
        SHOOT_STACK2,
        DRIVE_SHOOTPOS_ENDPOS,
        DRIVE_STACK3POS_SHOOTPOS,
        SHOOT_STACK3
    }

    PathState pathState;

    private boolean keepRunning = true;

    private final Pose startPose = new Pose(
            19.287,
            121.465,
            Math.toRadians(135)
    );

    private final Pose shootPose = new Pose(
            60,
            81,
            Math.toRadians(135)
    );

    private final Pose stack1Pose = new Pose(
            16,
            81,
            Math.toRadians(0)
    );
    private final Pose stack2Pose = new Pose(
            18,
            81,
            Math.toRadians(0)
    );
    private final Pose gateOpen1aPose = new Pose(71.937, 76.058, Math.toRadians(180));
    private final Pose gateOpen1bPose = new Pose(15.245, 70.5, Math.toRadians(180));

    private final Pose stack2aPose = new Pose(63.000, 54.000, Math.toRadians(0));
    private final Pose stack2bPose = new Pose(15.000, 57.000, Math.toRadians(0));

    private final Pose gateOpen2aPose = new Pose(69.798, 66.969, Math.toRadians(180));
    private final Pose gateOpen2bPose = new Pose(15.245, 68.466, Math.toRadians(180));

    private final Pose stack3aPose = new Pose(66.000, 30.000, Math.toRadians(0));
    private final Pose stack3bPose = new Pose(15.000, 31.000, Math.toRadians(0));
    private final Pose endPose = new Pose(39, 78, Math.toRadians(135));

    private PathChain driveStartPosShootPos,
            driveStack1PosEndPos,
            driveGateOpen1PosShootPos,
            driveGateOpen1PosEndPos,
            driveStack2PosEndPos,
            driveGateOpen2PosEndPos,
            driveGateOpen2PosShootPos,
            driveStack2PosShootPos,
            driveStack3PosEndPos,
            driveStack3PosShootPos,
            driveShootPosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveStack1PosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stack1Pose))
                .setLinearHeadingInterpolation(stack1Pose.getHeading(), stack1Pose.getHeading())
                .build();

        driveGateOpen1PosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(stack1Pose, gateOpen1aPose, gateOpen1bPose))
                .setLinearHeadingInterpolation(stack1Pose.getHeading(), gateOpen1bPose.getHeading())
                .build();

        driveGateOpen1PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gateOpen1bPose, shootPose))
                .setLinearHeadingInterpolation(gateOpen1bPose.getHeading(), shootPose.getHeading())
                .build();

        driveStack2PosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, stack2aPose, stack2bPose))
                .setLinearHeadingInterpolation(stack2aPose.getHeading(), stack2bPose.getHeading())
                .build();

        driveGateOpen2PosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(stack2bPose, gateOpen2aPose, gateOpen2bPose))
                .setLinearHeadingInterpolation(stack1Pose.getHeading(), gateOpen1bPose.getHeading())
                .build();

        driveGateOpen2PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gateOpen2bPose, shootPose))
                .setLinearHeadingInterpolation(gateOpen1bPose.getHeading(), shootPose.getHeading())
                .build();

        driveStack2PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(stack2bPose, shootPose))
                .setLinearHeadingInterpolation(stack2bPose.getHeading(), shootPose.getHeading())
                .build();

        driveStack3PosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, stack3aPose, stack3bPose))
                .setLinearHeadingInterpolation(stack3aPose.getHeading(), stack3bPose.getHeading())
                .build();

        driveStack3PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(stack3bPose, shootPose))
                .setLinearHeadingInterpolation(stack3bPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset the timer & make new state

                break;

            case SHOOT_PRELOAD:
                // check is follower done it's path?
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveStack1PosEndPos, true);
                        setPathState(PathState.DRIVE_STACK1POS_GATE1POS);
                    }
                }
                break;

            case DRIVE_STACK1POS_GATE1POS:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.followPath(driveGateOpen1PosEndPos, true);
                    setPathState(PathState.DRIVE_GATE1POS_SHOOTPOS); // reset the timer & make new state
                }
                break;

            case DRIVE_GATE1POS_SHOOTPOS:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.followPath(driveGateOpen1PosShootPos, true);
                    setPathState(PathState.SHOOT_STACK1); // reset the timer & make new state
                }
                break;

            case SHOOT_STACK1:
                // check is follower done it's path?
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveStack2PosEndPos, true);
                        setPathState(PathState.DRIVE_STACK2POS_GATE2POS);
                    }
                }
                break;

            case DRIVE_STACK2POS_GATE2POS:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.followPath(driveGateOpen2PosEndPos, true);
                    setPathState(PathState.DRIVE_GATE2POS_SHOOTPOS); // reset the timer & make new state
                }
                break;

            case DRIVE_GATE2POS_SHOOTPOS:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.followPath(driveGateOpen2PosShootPos, true);
                    setPathState(PathState.SHOOT_STACK2); // reset the timer & make new state
                }
                break;

            case SHOOT_STACK2:
                // check is follower done it's path?
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition

                        follower.followPath(driveStack3PosEndPos, true);
                        setPathState(PathState.DRIVE_STACK3POS_SHOOTPOS);
                    }
                }
                break;
            case DRIVE_STACK3POS_SHOOTPOS:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.followPath(driveStack3PosShootPos, true);
                    setPathState(PathState.SHOOT_STACK3); // reset the timer & make new state
                }
                break;
            case SHOOT_STACK3:
                // check is follower done it's path?
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosEndPos, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                    }
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                // all done!
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    stopSubsystems();
                    telemetry.addLine("Done all Paths");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //add in any other init mechanisms
        feederStopper.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        belt = new Belt(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        shooterSpeed = shooter.getShooterSpeed(INSIDE_FLAG, voltageSensor.getVoltage());
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        robotStateUpdate();
    }

    public void robotStateUpdate() {
        shooter.run(shooterSpeed);
        intake.run(INTAKE_80P_POWER);
        belt.run();
    }

    public void stopSubsystems() {
        shooter.stop();
        intake.stop();
        belt.stop();
        keepRunning = false;
    }

    @Override
    public void loop() {
        follower.update();
        feederStopper.update();
        statePathUpdate();
        if (keepRunning) {
            robotStateUpdate();
        }

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shooter Speed", shooterSpeed);
    }

}