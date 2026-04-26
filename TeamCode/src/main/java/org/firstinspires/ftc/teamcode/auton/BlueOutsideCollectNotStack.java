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


// This will attempt 3rd stack and human player artifacts
@Autonomous(name = "Blue Outside Collect-NO Stack", group = "blue", preselectTeleOp = "TeleOp BLUE Decode Drive Game")
public class BlueOutsideCollectNotStack extends OpMode implements DriveParams {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    // ---------- FEEDER FLYWHEEL SETUP ----------
    private FeederFlywheelLogic  feederStopper = new FeederFlywheelLogic();
    private Shooter shooter;
    private Intake intake;
    private Belt belt;
    private boolean shotsTriggered = false;
    private VoltageSensor voltageSensor;
    private double shooterSpeed = SHOOTER_65P_POWER;
    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_STACK1POS_SHOOTPOS,
        SHOOT_STACK1,
        DRIVE_STACK2POS_SHOOTPOS,
        SHOOT_STACK2,
        DRIVE_COLLECTPOS_SHOOTPOS,
        SHOOT_COLLECT,
        DRIVE_SHOOTPOS_ENDPOS
    }

    PathState pathState;

    private boolean keepRunning = true;

    private final Pose startPose = new Pose(
            59.178,
            9.841,
            Math.toRadians(113.5) );

    private final Pose shootPose = new Pose(
            59.178,       //59
               9.841,        // 11
            Math.toRadians(113.5)
    );

    //private final Pose stack1aPose = new Pose(90.018, 37.494, Math.toRadians(0));
    //private final Pose stack1bPose = new Pose(15.156, 37.088, Math.toRadians(0) );
    //private final Pose stack1aRetPose = new Pose(53.934, 41.410, Math.toRadians(112) );
    //private final Pose stack2aPose = new Pose(34.924, 68.461, Math.toRadians(19));
    //private final Pose stack2bPose = new Pose(2.545, 28.583, Math.toRadians(19));
    //private final Pose stack2cPose = new Pose(6.875, 8.114, Math.toRadians(19));
    //private final Pose stack2RetPose = new Pose(35.170, 39.238, Math.toRadians(112));
    private final Pose collectaPose = new Pose(65.490, 24.253, Math.toRadians(40));
    private final Pose collectbPose = new Pose(21.618, 17.603, Math.toRadians(40));//21.264 21.679
    private final Pose collectcPose = new Pose(4.562, 28.600, Math.toRadians(40));
    private final Pose collectdPose = new Pose(7.755, 6.914, Math.toRadians(40));

    private final Pose collectRetaPose = new Pose(35.448, 16.569, Math.toRadians(40));
    private final Pose collectRetbPose = new Pose(51.664, 14.704, Math.toRadians(40));
    private final Pose endPose = new Pose(50.320,35.274, Math.toRadians(113.5));

    private PathChain driveStartPosShootPos,
                driveStack1PosEndPos,
                driveStack1PosShootPos,
                driveStack2PosEndPos,
                driveStack2PosShootPos,
                driveCollectPosEndPos,
                driveCollectPosShootPos,
                driveShootPosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
//        driveStack1PosEndPos = follower.pathBuilder()
//                .addPath(new BezierCurve(startPose, stack1aPose, stack1bPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), stack1bPose.getHeading())
//                .build();
//
//        driveStack1PosShootPos = follower.pathBuilder()
//                .addPath(new BezierCurve(stack1bPose, stack2aPose, shootPose))
//                .setLinearHeadingInterpolation(stack1bPose.getHeading(), shootPose.getHeading())
//                .build();
//
//        driveStack2PosEndPos = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, stack2aPose, stack2bPose, stack2cPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), stack2bPose.getHeading())
//                .build();
//
//        driveStack2PosShootPos = follower.pathBuilder()
//                .addPath(new BezierCurve(stack2bPose, stack2RetPose, shootPose))
//                .setLinearHeadingInterpolation(stack2bPose.getHeading(), shootPose.getHeading())
//                .build();
        driveCollectPosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, collectaPose, collectbPose, collectcPose, collectdPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectdPose.getHeading())
                .build();

        driveCollectPosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(collectcPose, collectRetaPose, collectRetbPose, shootPose))
                .setLinearHeadingInterpolation(collectcPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case STARTPOS_SHOOTPOS:
                if (pathTimer.getElapsedTimeSeconds() > READY_STATE_WAIT_TIME) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
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
                        follower.followPath(driveCollectPosEndPos, true);
                        setPathState(PathState.DRIVE_COLLECTPOS_SHOOTPOS);
                    }
                }
                break;

//            case DRIVE_STACK1POS_SHOOTPOS:
//                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
//                    follower.followPath(driveStack1PosShootPos, true);
//                    setPathState(PathState.SHOOT_STACK1); // reset the timer & make new state
//                }
//                break;

//            case SHOOT_STACK1:
//                // check is follower done it's path?
//                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
//                    // requested shots yet?
//                    if (!shotsTriggered) {
//                        feederStopper.fireShots(true);
//                        shotsTriggered = true;
//                    } else if (shotsTriggered && !feederStopper.isBusy()) {
//                        // shots are done free to transition
//                        follower.followPath(driveStack2PosEndPos, true);
//                        setPathState(PathState.DRIVE_STACK2POS_SHOOTPOS);
//                    }
//                }
//                break;

//            case DRIVE_STACK2POS_SHOOTPOS:
//                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
//                    follower.followPath(driveStack2PosShootPos, true);
//                    setPathState(PathState.SHOOT_STACK2); // reset the timer & make new state
//                }
//                break;

//            case SHOOT_STACK2:
//                // check is follower done it's path?
//                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
//                    // requested shots yet?
//                    if (!shotsTriggered) {
//                        feederStopper.fireShots(true);
//                        shotsTriggered = true;
//                    } else if (shotsTriggered && !feederStopper.isBusy()) {
//                        // shots are done free to transition
//                        follower.followPath(driveCollectPosEndPos, true);
//                        setPathState(PathState.DRIVE_COLLECTPOS_SHOOTPOS);
//                    }
//                }
//                break;
            case DRIVE_COLLECTPOS_SHOOTPOS:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.followPath(driveCollectPosShootPos, true);
                    setPathState(PathState.SHOOT_COLLECT); // reset the timer & make new state
                }
                break;

            case SHOOT_COLLECT:
                // check is follower done it's path?
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition
                        if (opModeTimer.getElapsedTimeSeconds() < COLLECT_SHOOT_DEADLINE){
                            follower.followPath(driveCollectPosEndPos, true);
                            setPathState(PathState.DRIVE_COLLECTPOS_SHOOTPOS);
                        }else{
                            follower.followPath(driveShootPosEndPos, true);
                            setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                        }
                    }
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                // all done!
                if (!follower.isBusy()) {
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
        pathState = PathState.STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TODO add in any other init mechanisms
        feederStopper.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        belt = new Belt(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        //follower.setMaxPower(0.9);
        shooterSpeed = shooter.getShooterSpeed(OUTSIDE_FLAG, voltageSensor.getVoltage());
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        robotStateUpdate();
    }

    public void robotStateUpdate() {
        shooter.run(shooterSpeed);    // >13.5v -> 60, > 12.85; 13.21v -> 62; 14.32v  - 58; 13.17 ->62
        intake.run(INTAKE_80P_POWER);
        belt.run();
    }

    public void stopSubsystems(){
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