package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.geometry.*;
import com.pedropathing.util.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.DriveParams;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "Blue Outside Collection & Shoot", group = "Pathing")
public class BlueOutsideCollectionAndShoot extends OpMode implements DriveParams {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private FeederFlywheelLogic feederStopper = new FeederFlywheelLogic();
    private Shooter shooter;
    private Intake intake;
    private Belt belt;
    private boolean shotsTriggered = false;

    private OpenCvWebcam webcam;
    private ArtifactDetectionPipeline pipeline;

    // Autonomous duration = 30 seconds; collect/shoot for first 25, park in last 5
    private static final double COLLECT_SHOOT_DEADLINE = 25.0;
    private static final double AUTON_DURATION = 30.0;

    // Camera frame dimensions
    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;

    // Field mapping: convert pixel offset from camera center to field distance estimate
    // These values need tuning on the actual field
    private static final double PIXELS_TO_INCHES_X = 0.15;
    private static final double PIXELS_TO_INCHES_Y = 0.15;
    private static final double DETECTION_FORWARD_OFFSET = 18.0; // inches forward from robot to detected artifact

    public enum PathState {
        SCANNING,
        DRIVE_TO_ARTIFACT,
        COLLECTING,
        DRIVE_TO_SHOOT_POS,
        SHOOTING,
        DRIVE_TO_PARK,
        PARKED
    }

    private PathState pathState;

    private final Pose startPose = new Pose(59, 10, Math.toRadians(109));
    private final Pose shootPose = new Pose(59, 10, Math.toRadians(109));
    private final Pose parkPose = new Pose(48, 36, Math.toRadians(109));

    private Pose detectedArtifactPose = null;
    private PathChain currentPath;

    // Collect timer for intake dwell time at artifact location
    private Timer collectTimer;
    private static final double COLLECT_DWELL_TIME = 1.5; // seconds to run intake at artifact

    public void buildPathToArtifact(Pose target) {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()),
                        target))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build();
    }

    public void buildPathToShootPos() {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()),
                        shootPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading())
                .build();
    }

    public void buildPathToPark() {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()),
                        parkPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
                .build();
    }

    /**
     * Converts a detected artifact pixel position to a field pose estimate
     * relative to the robot's current position and heading.
     */
    private Pose estimateArtifactFieldPose(double pixelCenterX, double pixelCenterY) {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading();

        // Pixel offset from camera center
        double pixelOffsetX = pixelCenterX - (CAMERA_WIDTH / 2.0);
        double pixelOffsetY = (CAMERA_HEIGHT / 2.0) - pixelCenterY; // invert Y (pixel Y increases downward)

        // Convert pixel offset to local robot-frame inches
        double localLateral = pixelOffsetX * PIXELS_TO_INCHES_X;
        double localForward = DETECTION_FORWARD_OFFSET + (pixelOffsetY * PIXELS_TO_INCHES_Y);

        // Rotate into field frame using robot heading
        double fieldDx = localForward * Math.cos(robotHeading) - localLateral * Math.sin(robotHeading);
        double fieldDy = localForward * Math.sin(robotHeading) + localLateral * Math.cos(robotHeading);

        return new Pose(robotX + fieldDx, robotY + fieldDy, robotHeading);
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }

    public void statePathUpdate() {
        double elapsed = opModeTimer.getElapsedTimeSeconds();

        // If past 25 seconds, park regardless of current state
        if (elapsed >= COLLECT_SHOOT_DEADLINE && pathState != PathState.DRIVE_TO_PARK && pathState != PathState.PARKED) {
            stopSubsystems();
            buildPathToPark();
            follower.followPath(currentPath, true);
            setPathState(PathState.DRIVE_TO_PARK);
            return;
        }

        switch (pathState) {
            case SCANNING:
                // Check pipeline for detected artifacts
                ArtifactDetectionPipeline.DetectionResult result = pipeline.getLatestDetection();
                if (result != null && result.detected) {
                    detectedArtifactPose = estimateArtifactFieldPose(result.centerX, result.centerY);
                    buildPathToArtifact(detectedArtifactPose);
                    intake.run(INTAKE_80P_POWER);
                    belt.run();
                    follower.followPath(currentPath, true);
                    setPathState(PathState.DRIVE_TO_ARTIFACT);
                }
                // If nothing detected, keep scanning (robot stays at current position)
                break;

            case DRIVE_TO_ARTIFACT:
                if (!follower.isBusy()) {
                    collectTimer = new Timer();
                    collectTimer.resetTimer();
                    setPathState(PathState.COLLECTING);
                }
                break;

            case COLLECTING:
                // Dwell at artifact location with intake running
                intake.run(INTAKE_80P_POWER);
                belt.run();
                if (collectTimer.getElapsedTimeSeconds() >= COLLECT_DWELL_TIME) {
                    buildPathToShootPos();
                    follower.followPath(currentPath, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS);
                }
                break;

            case DRIVE_TO_SHOOT_POS:
                if (!follower.isBusy()) {
                    // Start shooter flywheel spin-up during return path
                    shooter.run(SHOOTER_55P_POWER);
                    intake.stop();
                    setPathState(PathState.SHOOTING);
                }
                break;

            case SHOOTING:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        feederStopper.fireShots(true);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !feederStopper.isBusy()) {
                        // Shooting done, go back to scanning for more artifacts
                        shooter.stop();
                        setPathState(PathState.SCANNING);
                    }
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    stopSubsystems();
                    setPathState(PathState.PARKED);
                }
                break;

            case PARKED:
                // Done
                break;

            default:
                break;
        }
    }

    private void stopSubsystems() {
        shooter.stop();
        intake.stop();
        belt.stop();
    }

    @Override
    public void init() {
        pathState = PathState.SCANNING;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        collectTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        feederStopper.init(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        belt = new Belt(hardwareMap);

        follower.setPose(startPose);
        follower.setMaxPower(0.5);

        // Initialize webcam with EasyOpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "WebCam1"), cameraMonitorViewId);

        pipeline = new ArtifactDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.addLine("BlueOutsideCollectionAndShoot initialized");
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.SCANNING);
        shooter.run(SHOOTER_55P_POWER);
    }

    @Override
    public void loop() {
        follower.update();
        feederStopper.update();
        statePathUpdate();

        telemetry.addData("State", pathState.toString());
        telemetry.addData("Time", "%.1f", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        ArtifactDetectionPipeline.DetectionResult det = pipeline.getLatestDetection();
        if (det != null && det.detected) {
            telemetry.addData("Detected", det.color);
            telemetry.addData("Pixel", "(%.0f, %.0f)", det.centerX, det.centerY);
            telemetry.addData("Area", det.area);
        } else {
            telemetry.addData("Detected", "None");
        }
    }

    @Override
    public void stop() {
        stopSubsystems();
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }

    /**
     * OpenCV pipeline that detects purple and green artifacts using HSV color filtering.
     */
    static class ArtifactDetectionPipeline extends OpenCvPipeline {

        // HSV thresholds for purple artifacts
        private static final Scalar PURPLE_LOW = new Scalar(120, 50, 50);
        private static final Scalar PURPLE_HIGH = new Scalar(160, 255, 255);

        // HSV thresholds for green artifacts
        private static final Scalar GREEN_LOW = new Scalar(35, 50, 50);
        private static final Scalar GREEN_HIGH = new Scalar(85, 255, 255);

        // Minimum contour area to count as a valid detection (filters noise)
        private static final double MIN_CONTOUR_AREA = 500;

        private volatile DetectionResult latestDetection = null;

        // Reusable Mats to reduce GC pressure
        private final Mat hsv = new Mat();
        private final Mat purpleMask = new Mat();
        private final Mat greenMask = new Mat();
        private final Mat hierarchy = new Mat();

        static class DetectionResult {
            boolean detected;
            double centerX;
            double centerY;
            double area;
            String color;

            DetectionResult(boolean detected, double centerX, double centerY, double area, String color) {
                this.detected = detected;
                this.centerX = centerX;
                this.centerY = centerY;
                this.area = area;
                this.color = color;
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create masks for purple and green
            Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, purpleMask);
            Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, greenMask);

            // Find the largest contour across both colors
            double bestArea = 0;
            double bestCx = 0;
            double bestCy = 0;
            String bestColor = "none";

            // Check purple contours
            List<MatOfPoint> purpleContours = new ArrayList<>();
            Imgproc.findContours(purpleMask, purpleContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : purpleContours) {
                double area = Imgproc.contourArea(contour);
                if (area > bestArea && area > MIN_CONTOUR_AREA) {
                    Moments m = Imgproc.moments(contour);
                    if (m.m00 > 0) {
                        bestArea = area;
                        bestCx = m.m10 / m.m00;
                        bestCy = m.m01 / m.m00;
                        bestColor = "purple";
                    }
                }
                contour.release();
            }

            // Check green contours
            List<MatOfPoint> greenContours = new ArrayList<>();
            Imgproc.findContours(greenMask, greenContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : greenContours) {
                double area = Imgproc.contourArea(contour);
                if (area > bestArea && area > MIN_CONTOUR_AREA) {
                    Moments m = Imgproc.moments(contour);
                    if (m.m00 > 0) {
                        bestArea = area;
                        bestCx = m.m10 / m.m00;
                        bestCy = m.m01 / m.m00;
                        bestColor = "green";
                    }
                }
                contour.release();
            }

            if (bestArea > 0) {
                latestDetection = new DetectionResult(true, bestCx, bestCy, bestArea, bestColor);
                // Draw crosshair on detected artifact for camera preview
                Imgproc.circle(input, new Point(bestCx, bestCy), 10, new Scalar(255, 0, 0), 2);
                Imgproc.putText(input, bestColor, new Point(bestCx + 15, bestCy),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 1);
            } else {
                latestDetection = new DetectionResult(false, 0, 0, 0, "none");
            }

            return input;
        }

        public DetectionResult getLatestDetection() {
            return latestDetection;
        }
    }
}
