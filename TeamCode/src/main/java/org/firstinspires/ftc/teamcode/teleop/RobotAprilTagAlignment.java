package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.DriveParams;

import java.util.List;


@TeleOp(name="TeleOp Decode Drive Game", group = "1")
public class RobotAprilTagAlignment extends LinearOpMode implements DriveParams{

    // create motor objects and initialize to null
    private DcMotor leftFrontMotor  = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor   = null;
    private DcMotor rightBackMotor  = null;
    private DcMotor beltMotor = null;

    private DcMotor encoderLeft = null;
    private DcMotor encoderRight = null;
    private DcMotor encoderAux = null;
    private DcMotor shooterRotateMotor  = null;
    private DcMotor intakeRotateMotor  = null;
    private Servo   shooterAngle = null;
    private Servo pusher = null;
    private Servo stopper = null;
    private Servo alignInd = null;
    private CRServo feeder = null;
    private double shooterPosition = 0;
    private double  shooterSpeed = 0.65;
    private boolean alreadyTriggeredUp = false;
    private boolean alreadyTriggeredDown = false;
    private VoltageSensor voltageSensor;
    private Limelight3A limelight;
    private boolean isAligning = false;
    private boolean previousButtonState = false;
    private double driveSpeed,
            currentSpeedCap,
            currentShooterSpeed,
            currentIntakeSpeed,
            forward,
            strafe,
            turn  = 0.0;


    //right Motor position
    //private static double ROTATE_ARM_UP = 0.0;
    //private static double ROTATE_ARM_FLOOR = 0.50;
    //left Motor position
    private static final double NO_POWER = 0.0;
    private static final double MOVE_THRESHOLD = 0.30;


    double L = 33.8;       // distance between left and right encoders in cm
    double B = 11.5;        // distance between encoder 1 and 2 axis to encoder 3
    double R = 2.4;         // wheel radius in cm
    double N = 2000;        // Number of ticks per rotation
    double cm_per_tick = 2.0 *  Math.PI * R / N;

    int currentRightPosition = 0;
    int currentLeftPosition = 0;
    int currentAuxPosition = 0;

    int oldRightPosition = 0;
    int oldLeftPosition = 0;
    int oldAuxPosition = 0;

    double localPosX = 213;
    double localPosY = 102;
    double timer_ms, localPosH = 0;
    int maxValueL, maxValueR, old_dn1, old_dn2, dn1, dn2, dn3 = 0;

    private boolean speedCapTogglePrev, shooterCapTogglePrev  = false;
    private boolean isShooterTriggered = false;
    private boolean isIntakeTriggered = false;

    double[] stepSizes = {0.1, 0.001, 0.0001};
    int stepIndex = 1;
    // -------------------- PD controller --------------------
    double kP           = 0.002;
    double error        = 0;
    double lastError    = 0;
    double goalX        = 0; // offset here
    double angleTolerance = 0.4;
    double kD           = 0.0001;
    double curTime      = 0;
    double lastTime     = 0;


    @Override public void runOpMode() {
        // Initialize Robot Configuration
        initConfig();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry.addData("Battery voltage >",  voltageSensor.getVoltage());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        closeStopper();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Battery voltage >", voltageSensor.getVoltage());

            if (gamepad1.y) {
                alreadyTriggeredDown = false;
                alreadyTriggeredUp = false;
            }

            checkDriveSpeedToggle(gamepad1.left_bumper);
            checkShooterSpeedToggle();

            forward = -gamepad1.left_stick_y * currentSpeedCap;  // Reduce drive rate to currentSpeedCap.
            strafe = -gamepad1.left_stick_x * currentSpeedCap;  // Reduce strafe rate to currentSpeedCap.
            turn = -gamepad1.right_stick_x * currentSpeedCap;  // Reduce turn rate to currentSpeedCap.
            if (forward != 0 || strafe != 0 || turn != 0) {
                alignTurnOff();
            }
            boolean currentButtonState = gamepad1.a;
            //
            if (gamepad1.dpad_up && !alreadyTriggeredUp) {
                if (shooterPosition < 0.06) {
                    shooterPosition += 0.02;
                }
                alreadyTriggeredUp = true;

            } else if (gamepad1.dpad_down && !alreadyTriggeredDown) {
                if (shooterPosition > 0) {
                    shooterPosition -= 0.02;
                }
                alreadyTriggeredDown = true;
            }
            //shooterAngle.setPosition(shooterPosition);
            if (gamepad2.left_bumper) {

                shooter(shooterSpeed);
            }
            if (gamepad2.right_stick_y == 0) {
                feeder.setPower(FEEDER_STOP_POWER);
            } else {
                feeder.setPower(FEEDER_FEED_POWER);
            }

            if (gamepad2.dpad_down) {
                closeStopper();
            } else if (gamepad2.dpad_up) {
                openStopper();
            } // else if (gamepad2.dpad_left){
            //    pusher.setPosition(0.2);

            //}
            intake(gamepad1.x);
            beltRun();
            //moveRobot(forward, strafe, turn);

            if (currentButtonState && !previousButtonState) {
                isAligning = !isAligning;
                if (isAligning) {
                    telemetry.addData("Mode", "AprilTag Alignment ACTIVE");
                } else {
                    telemetry.addData("Mode", "Manual Control");
                }
            }
            previousButtonState = currentButtonState;

            if (isAligning) {
                performAprilTagAlignmentNew();
            } else if (gamepad1.left_trigger > 0.3) {
                performAprilTagAlign(forward, strafe, turn);
            }else {
                manualDrive();
            }
            sendTelemetry(true);
        }

    }

    private void sendTelemetry(boolean update) {
        telemetry.addData("LRS", "%6d    %6d    %6d  ", currentLeftPosition, currentRightPosition, currentAuxPosition);
        telemetry.addData("xyh", "%6.1f cm    %6.1f cm    %6.1f deg  ", localPosX, localPosY, Math.toDegrees(localPosH));
        telemetry.addData("Max L R", "%6d    %6d  ", maxValueL, maxValueR);
        telemetry.addData("Drive Speed>", currentSpeedCap);
        telemetry.addData("Shooter Speed>", shooterSpeed);
        telemetry.addData("Distance from AprilTag (inches)>", calculateDistanceFromAprilTag());

        if (update) telemetry.update();
    }
    /**
     *
     * @param speedToggleCurrent
     */
    private void checkDriveSpeedToggle(boolean speedToggleCurrent){

        if (speedToggleCurrent &&  (speedToggleCurrent != speedCapTogglePrev)) {
            if (currentSpeedCap == SPEED_CAP_MIN){
                currentSpeedCap = SPEED_CAP_MAX;
            }else {
                currentSpeedCap = SPEED_CAP_MIN;
            }
        }
        speedCapTogglePrev = speedToggleCurrent;
    }

    /**
     *
     */
    private void checkShooterSpeedToggle(){

        if (gamepad2.y){
            shooterSpeed = SHOOTER_70P_POWER;
        } else if (gamepad2.x) {
            shooterSpeed = SHOOTER_60P_POWER;
        }else if (gamepad2.b){
            shooterSpeed = SHOOTER_55P_POWER;
        }else if (gamepad2.a) {
            shooterSpeed = SHOOTER_65P_POWER;
        }else if (gamepad2.rightBumperWasPressed()){
            shooterSpeed += .01;
        }else if (gamepad2.leftBumperWasPressed()) {
            shooterSpeed -= .01;
        }
        shooter(shooterSpeed);
    }

    /**
     *
     *
     */
    private void initConfig(){

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "left_rear");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "right_rear");

        shooterRotateMotor  = hardwareMap.get(DcMotor.class, "shooter");
        intakeRotateMotor  = hardwareMap.get(DcMotor.class, "intake");
        beltMotor = hardwareMap.get(DcMotor.class, "belt");

        shooterAngle = hardwareMap.get(Servo.class, "shooter_angle");
        //pusher = hardwareMap.get(Servo.class, "pusher");
        stopper = hardwareMap.get(Servo.class, "stopper");
        alignInd = hardwareMap.get(Servo.class, "alignled");
        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setDirection(DcMotor.Direction.FORWARD);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        currentSpeedCap = SPEED_CAP_MIN;
        currentIntakeSpeed = 0.50;
        shooterAngle.setPosition(SHOOTER_ANGLE_05);

        // update log on to driver station
        telemetry.update();

    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;


        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));

        double max = Math.max(Math.abs(leftFrontPower),
                        Math.max(Math.abs(leftBackPower),
                        Math.max(Math.abs(rightFrontPower),
                        Math.abs(rightBackPower))));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     Odometry calculation math
     */
    public void checkOdometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        dn1 = currentLeftPosition  - oldLeftPosition;
        dn2 = currentRightPosition - oldRightPosition;
        dn3 = currentAuxPosition - oldAuxPosition;

        // the robot has moved and turned a tiny bit between two measurements:
        double dx = cm_per_tick * ((dn1+dn2) / 2.0);
        double dy = cm_per_tick * (dn3 + ((dn2-dn1) / 2.0));
        double dtheta = cm_per_tick * ((dn2-dn1) / (L));

        localPosX += dx;
        localPosY += dy;

        // small movement of the robot gets added to the field coordinate system:
        localPosH += dtheta / 2;
        localPosX += dx * Math.cos(localPosH) - dy * Math.sin(localPosH);
        localPosY += dx * Math.sin(localPosH) + dy * Math.cos(localPosH);
        localPosH += dtheta / 2;
    }

    /**
     *
     */
    private void shooter(double speed){
        shooterRotateMotor.setPower(speed); // currentShooterSpeed);
    }

    /**
     *
     */
    private void intake(boolean reverse){
        if (reverse) {
            intakeRotateMotor.setPower(-INTAKE_SPEED_CAP);
        } else {
            intakeRotateMotor.setPower(INTAKE_SPEED_CAP);
        }
    }

    /**
     *
     */
    private void beltRun(){
        beltMotor.setPower(BELT_SPEED_CAP);
    }

    /**
     *
     */
    private void performAprilTagAlignment() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            //
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int tagID = fr.getFiducialId();
                    // Check if AprilTag ID is 20 or 24
                    if (tagID == 20 || tagID == 24) {
                        // Get offsets from center -> use these values for robot detection relative to field
                        double tx = fr.getTargetXDegrees(); // Horizontal offset
                        double ty = fr.getTargetYDegrees(); // Vertical offset
                        double ta = fr.getTargetArea(); // Can be used for distance estimation

                        // Calculate motor powers using proportional control
                        double strafePower = Range.clip(tx * KP_STRAFE, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                        double forwardPower = Range.clip(-ty * KP_FORWARD, -MAX_FORWARD_POWER, MAX_FORWARD_POWER);
                        double turnPower = Range.clip(tx * KP_TURN, -MAX_TURN_POWER, MAX_TURN_POWER);
                        boolean isAligned = false;
                        // Check if aligned
                        //boolean isAligned =   Math.abs(tx) < ALIGNMENT_THRESHOLD_X; // &&
                                //Math.abs(ty) < ALIGNMENT_THRESHOLD_Y;
                        if (tagID == 20 && (tx < 2.0 && tx > 1.0)) {
                            isAligned = true;
                        }else if (tagID == 24 && (tx < 1.0 && tx > 0.5)){
                            isAligned = true;
                        }
                        if (isAligned) {
                            // Stop motors when aligned
                            alignTurnOn();
                            setMotorPowers(0, 0, 0);
                            telemetry.addData("Status", "ALIGNED!");
                        } else {
                            // Apply alignment movements
                            // Use strafe to center horizontally and turn to face the tag
                            setMotorPowers(0, 0, turnPower); //strafePower, forwardPower, turnPower);

                            telemetry.addData("ID", fr.getFiducialId());
                            telemetry.addData("Status", "Aligning...");
                            telemetry.addData("TX (horizontal)", "%.2f", tx);
                            telemetry.addData("TY (vertical)", "%.2f", ty);
                            telemetry.addData("Area", "%.2f", ta);
                            telemetry.addData("turnPower", "%.2f", turnPower);
                        }

                        telemetry.addData("AprilTag ID", fr.getFiducialId());

                    } else {
                        // No AprilTag detected
                        setMotorPowers(0, 0, 0);
                        telemetry.addData("Status", "No AprilTag detected");
                    }
                }
            } else {
                // No valid result
                setMotorPowers(0, 0, 0);
                telemetry.addData("Status", "Waiting for Limelight data...");
            }
        }
        telemetry.addData("Mode", "AprilTag Alignment (Press A to exit)");
    }

    /**
     *
     */
    private void manualDrive() {
        // Standard mecanum drive control
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x * 1.1; // Strafe (multiplier to counteract imperfect strafing)
        double rx = gamepad1.right_stick_x; // Rotation

        // Calculate motor powers for mecanum drive
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);

        telemetry.addData("Mode", "Manual Control (Press A for alignment)");
        telemetry.addData("Left Stick", "Y: %.2f, X: %.2f", y, x);
        telemetry.addData("Right Stick", "X: %.2f", rx);
    }

    /**
     *
     * @param strafe
     * @param forward
     * @param turn
     */
    private void setMotorPowers(double strafe, double forward, double turn) {
        // Mecanum drive kinematics
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower),
                Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);

    }

    private void alignTurnOn(){
        alignInd.setPosition(0.388);
    }
    private void alignTurnOff(){
        alignInd.setPosition(0);
    }

    private void performAprilTagAlignmentNew() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult targetTag = null;

                // Find tag 20 or 24
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                        targetTag = fr;
                        break;
                    }
                }

                if (targetTag != null) {
                    // Get horizontal offset - this tells us how much to turn
                    double tx = targetTag.getTargetXDegrees();  // Horizontal offset in degrees

                    // Calculate turn power using proportional control
                    // Positive tx = tag is to the right, so turn right (positive)
                    double turnPower = Range.clip(tx * KP_TURN, -MAX_TURN_POWER, MAX_TURN_POWER);

                    // Add deadband to prevent micro-adjustments
                    if (Math.abs(turnPower) < 0.05) {
                        turnPower = 0;
                    }

                    // Check if aligned (facing the tag)
                    boolean isAligned = Math.abs(tx) < ALIGNMENT_THRESHOLD_X;

                    if (isAligned) {
                        // Stop and indicate alignment
                        alignTurnOn();
                        setMotorPowers(0, 0, 0);
                        telemetry.addData("Status", "✓ ALIGNED!");
                        telemetry.addData("Offset", "%.2f°", tx);
                    } else {
                        // Turn to face the tag
                        alignTurnOff();
                        setMotorPowers(0, 0, turnPower);

                        telemetry.addData("Status", "Aligning...");
                        telemetry.addData("Horizontal Offset", "%.2f°", tx);
                        telemetry.addData("Turn Power", "%.2f", turnPower);
                    }

                    telemetry.addData("AprilTag ID", targetTag.getFiducialId());

                } else {
                    // Tag 20 or 24 not found
                    alignTurnOff();
                    setMotorPowers(0, 0, 0);
                    telemetry.addData("Status", "Target tag (20/24) not visible");
                }

            } else {
                // No tags detected
                alignTurnOff();
                setMotorPowers(0, 0, 0);
                telemetry.addData("Status", "No AprilTags detected");
            }

        } else {
            // No valid Limelight result
            alignTurnOff();
            setMotorPowers(0, 0, 0);
            telemetry.addData("Status", "Waiting for Limelight data...");
        }

        telemetry.addData("Mode", "AprilTag Alignment (Press A to exit)");
    }

    private void closeStopper(){
        stopper.setPosition(GATE_CLOSED_POSITION);
        //sleep(2);
        //pusher.setPosition(PUSHER_RETRACT_POSITION);
    }

    private void openStopper(){
       // pusher.setPosition(PUSHER_PUSH_POSITION);
        stopper.setPosition(GATE_OPEN_POSITION);
        feeder.setPower(FEEDER_FEED_POWER);
    }

    /**
     * Calculate robot distance in inches from the april tag using the limelight camera
     * @return distance in inches
     */
    private double calculateDistanceFromAprilTag() {
        double distance = -1.0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                // Usually we want the first or specific target tag (20 or 24)
                LLResultTypes.FiducialResult targetTag = null;
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                        targetTag = fr;
                        double StrafeDistance_3D = fr.getRobotPoseTargetSpace().getPosition().y;
                   //     telemetry.addData("Fiducial " + fr.getFiducialId() , "is " + StrafeDistance_3D + " meters away");
                        break;
                    }
                }

                if (targetTag != null) {
                    // Get the camera-space transform
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ty = result.getTy(); // How far up or down the target is (degrees)
                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)
                   // telemetry.addData("Target X", tx);
                   // telemetry.addData("Target Y", ty);
                   // telemetry.addData("Target Area", ta);

                    if (result != null && result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                        if (botpose != null) {
                            double x = botpose.getPosition().x;
                            double y = botpose.getPosition().y;
                   //         telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                        }
                    }
                }
            }
        }
        return distance;
    }

    private void performAprilTagAlign(double strafe, double forward, double turn) {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult targetTag = null;

                // Find tag 20 or 24
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                        targetTag = fr;
                        break;
                    }
                }

                if (targetTag != null) {
                    // Get horizontal offset - this tells us how much to turn
                    double tx = targetTag.getTargetXDegrees();  // Horizontal offset in degrees

                    error = goalX - tx; // tx

                    if (Math.abs(error) < angleTolerance) {
                        turn = 0;
                    } else {
                        double pTerm = error * kP;

                        curTime = getRuntime();
                        double dT = curTime - lastTime;
                        double dTerm = ((error - lastError) / dT) * kD;

                        turn = Range.clip(pTerm + dTerm, -0.4, 0.4);
                        lastError = error;
                        lastTime = curTime;
                    }
                } else {
                    lastTime = getRuntime();
                    lastError = 0;
                }
            } else {
                lastError = 0;
                lastTime = getRuntime();
            }
            // drive our motors!
            setMotorPowers(forward, strafe, turn);


            // update P and D on the fly
            // 'B' button cycles through the different step sizes for tuning precision.
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length; // wraps index back to 0
            }

            // D-pad left/right adjusts the P gain.
            if (gamepad1.dpadLeftWasPressed()) {
                kP -= stepSizes[stepIndex];
            }
            if (gamepad1.dpadRightWasPressed()) {
                kP += stepSizes[stepIndex];
            }

            // D-pad up/down adjusts the D gain.
            if (gamepad1.dpadUpWasPressed()) {
                kD += stepSizes[stepIndex];
            }
            if (gamepad1.dpadDownWasPressed()) {
                kD -= stepSizes[stepIndex];
            }
        }
        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", kD);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }

}
