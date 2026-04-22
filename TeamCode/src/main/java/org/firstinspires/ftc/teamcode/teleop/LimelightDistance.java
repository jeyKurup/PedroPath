package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Limelight Distance Test", group="2")
public class LimelightDistance extends OpMode {

    private Limelight3A limelight3A;

    private double CAMERA_HEIGHT_CM = 43.50;

    private double CAMERA_ANGLE_DEGREES = 9.25;

    private double GOAL_HEIGHT = 74.95;

    private double distance = 0;
    private double DST1 = 308.5;


    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {

        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            distance = getDistance(llResult.getTy());
            telemetry.addData("Distance from April Tag (CM): ", distance);
            telemetry.addData("Camera Angle (Ty):", llResult.getTy());
            telemetry.addData("Angle from known Distance (DST1): ", getAngleFromKnownDistance(DST1, llResult.getTy()));
        }else {
            telemetry.addData("No Valid Target Found", "**No Target**");
        }

    }

    public double getDistance(double ty) {
        double angleToTarget = CAMERA_ANGLE_DEGREES + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_CM;
        //telemetry.addData("Angle to the Camera (Total Angle): ", angleToTarget);

        return (heightDifference / Math.tan(Math.toRadians(angleToTarget)));
    }

    public double getAngleFromKnownDistance(double distance, double ty) {
        return (Math.toDegrees(Math.atan((GOAL_HEIGHT - CAMERA_HEIGHT_CM) / distance)) - ty);
    }
}
