package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.DriveParams;

import java.util.List;

@TeleOp(name = "Robot Align BLUE Outside", group = "Calibration")
public class TeleOpSetRobotPositionBlueOutside extends OpMode implements DriveParams {
    // Create an instance of the sensor
    GoBildaPinpointDriver pinpoint;
    private Servo alignInd = null;
    private Limelight3A limelight;
    private final int tagID = 20;

    @Override
    public void init() {
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 90, AngleUnit.DEGREES, 0));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Push your robot around to see it track");
        telemetry.addLine("Press A to reset the position");
        if (gamepad1.a) {
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 6.992, 140.610, AngleUnit.DEGREES, 90));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Desired Heading angle:", 22);
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        double angleTurned = Math.abs(pose2D.getHeading(AngleUnit.DEGREES));

        performAprilTagAlignment();

    }

    public void configurePinpoint() {
        alignInd = hardwareMap.get(Servo.class, "alignled");
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-6.625, -7.50, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    private void alignTurnOn(double color) {
        alignInd.setPosition(color);
    }

    private void alignTurnOff() {
        alignInd.setPosition(0);
    }

    private void alignTurnOn() {
        alignInd.setPosition(YELLOW_IND);
    }

    private void performAprilTagAlignment() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            //
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == tagID) {
                        // Get offsets from center -> use these values for robot detection relative to field
                        double tx = fr.getTargetXDegrees(); // Horizontal offset
                        double ty = fr.getTargetYDegrees(); // Vertical offset
                        double ta = fr.getTargetArea(); // Can be used for distance estimation

                        // Check if aligned
                        boolean isAligned = Math.abs(tx) < ALIGNMENT_THRESHOLD_X; // &&
                        //Math.abs(ty) < ALIGNMENT_THRESHOLD_Y;

                        if (isAligned) {
                            // Stop motors when aligned

                            alignTurnOn(BLUE_IND);
                            telemetry.addData("Status", "ALIGNED!");
                            telemetry.addData("ID", fr.getFiducialId());
                            telemetry.addData("Status", "Aligning...");
                            telemetry.addData("TX (horizontal)", "%.2f", tx);
                            telemetry.addData("TY (vertical)", "%.2f", ty);
                            telemetry.addData("Area", "%.2f", ta);
                        } else {
                            alignTurnOff();
                            // Apply alignment movements
                            // Use strafe to center horizontally and turn to face the tag
                            telemetry.addData("ID", fr.getFiducialId());
                            telemetry.addData("Status", "Aligning...");
                            telemetry.addData("TX (horizontal)", "%.2f", tx);
                            telemetry.addData("TY (vertical)", "%.2f", ty);
                            telemetry.addData("Area", "%.2f", ta);
                        }

                        telemetry.addData("AprilTag ID", fr.getFiducialId());

                    } else {
                        // No AprilTag detected
                        telemetry.addData("Status", "No AprilTag detected");
                    }
                }
            } else {
                // No valid result
                telemetry.addData("Status", "Waiting for Limelight data...");
            }
        }

    }

}

