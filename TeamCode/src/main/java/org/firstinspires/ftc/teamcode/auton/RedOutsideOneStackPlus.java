package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.DriveParams;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Red Outside 1 Stack+ - WORLDS", group = "red", preselectTeleOp = "TeleOp Decode Drive Game")
public class RedOutsideOneStackPlus extends OpMode implements DriveParams {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ---------- FEEDER FLYWHEEL SETUP ----------
    private FeederFlywheelLogic  feederStopper = new FeederFlywheelLogic();
    private Shooter shooter;
    private Intake intake;
    private Belt belt;
    private boolean shotsTriggered = false;

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
        DRIVE_SHOOTPOS_ENDPOS

    }

    PathState pathState;

    private boolean keepRunning = true;

    private final Pose startPose = new Pose(
            82.322,
            9.841,
            Math.toRadians(71) );

    private final Pose shootPose = new Pose(
            83.605,       //59
               10.960,        // 11
            Math.toRadians(71)
    );

    private final Pose stack1aPose = new Pose(52.996, 47.252, Math.toRadians(180));
    private final Pose stack1bPose = new Pose(126.344, 38.265, Math.toRadians(180) );

    private final Pose stack1aRetPose = new Pose(86.220, 46.121, Math.toRadians(71) );

    private final Pose stack2aPose = new Pose(131.530, 53.890, Math.toRadians(71));
    private final Pose stack2bPose = new Pose(134.439, 20.459, Math.toRadians(160));
    private final Pose stack2cPose = new Pose(132.365, 9.994, Math.toRadians(71));

    private final Pose stack2RetPose = new Pose(99.936, 63.130, Math.toRadians(71));

    private final Pose endPose = new Pose(86.765,35.898, Math.toRadians(71));

    private PathChain driveStartPosShootPos,
                driveStack1PosEndPos,
                driveStack1PosShootPos,
                driveStack2PosEndPos,
                driveStack2PosShootPos,
                driveShootPosEndPos;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStack1PosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, stack1aPose, stack1bPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), stack1bPose.getHeading())
                .build();

        driveStack1PosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(stack1bPose, stack2aPose, shootPose))
                .setLinearHeadingInterpolation(stack1bPose.getHeading(), shootPose.getHeading())
                .build();

        driveStack2PosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, stack2aPose, stack2bPose, stack2cPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stack2bPose.getHeading())
                .build();

        driveStack2PosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(stack2bPose, stack2RetPose, shootPose))
                .setLinearHeadingInterpolation(stack2bPose.getHeading(), shootPose.getHeading())
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
                if (!follower.isBusy()) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveStack1PosEndPos, true);
                        setPathState(PathState.DRIVE_STACK1POS_SHOOTPOS);
                    }
                }
                break;

            case DRIVE_STACK1POS_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveStack1PosShootPos, true);
                    setPathState(PathState.SHOOT_STACK1); // reset the timer & make new state
                }
                break;

            case SHOOT_STACK1:
                // check is follower done it's path?
                if (!follower.isBusy()) {
                    // requested shots yet?
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveStack2PosEndPos, true);
                        setPathState(PathState.DRIVE_STACK2POS_SHOOTPOS);
                    }
                }
                break;

            case DRIVE_STACK2POS_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveStack2PosShootPos, true);
                    setPathState(PathState.SHOOT_STACK2); // reset the timer & make new state
                }
                break;

            case SHOOT_STACK2:
                // check is follower done it's path?
                if (!follower.isBusy()) {
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

        // TODO add in any other init mechanisms
        feederStopper.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        belt = new Belt(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        follower.setMaxPower(0.7);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        robotStateUpdate();
    }

    public void robotStateUpdate() {
        shooter.run(SHOOTER_65P_POWER);    // >13.5v -> 62, > 12.85
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
    }
}