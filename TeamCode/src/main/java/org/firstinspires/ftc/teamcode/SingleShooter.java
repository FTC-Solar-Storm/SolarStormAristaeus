package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class SingleShooter {

    // Shooter motors
    public DcMotorEx motor;

    // Limelight and drivetrain
    private Limelight3A limelight;
    private TankDrive drive;
    private Telemetry telemetry;

    // Shooter state
    private double targetVelocity = 0;
    public boolean upToSpeed = false;

    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    // Auto-aim state
    private boolean autoAimEnabled = false;

    // BIOBUZZ AprilTag IDs
    private static final int MIN_TAG_ID = 30;
    private static final int MAX_TAG_ID = 45;

    // Auto-aim tuning
    private static final double TURN_KP = 0.035;
    private static final double MAX_TURN_POWER = 0.35;
    private static final double MIN_TURN_POWER = 0.08;
    private static final double TURN_TOLERANCE_DEGREES = 1.5;

    private static final double APPROACH_KP = 0.45;
    private static final double MAX_FORWARD_POWER = 0.35;
    private static final double MIN_FORWARD_POWER = 0.10;
    private static final double STOP_DISTANCE_METERS = 0.50;

    private int selectedTagId = -1;
    private double selectedTagDistance = -1;
    private double selectedTagAngle = 0;

    public SingleShooter(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            TankDrive drive,
            String name
    ) {

        this.telemetry = telemetry;
        this.drive = drive;

        // Initialize both flywheel motors.
        motor = hardwareMap.get(DcMotorEx.class, name);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure velocity PIDF.
        PIDFCoefficients pidf = new PIDFCoefficients(
                10,
                0,
                0,
                12
        );

        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        // Initialize Limelight 3A.
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // Pipeline 0 must be configured for BIOBUZZ AprilTags.
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // --------------------------------------------------
    // SHOOTER CONTROL
    // --------------------------------------------------

    /**
     * Adjusts the target flywheel velocity using the D-pad.
     * Call repeatedly while the shooter is enabled.
     */
    public void update(boolean dpadUp, boolean dpadDown) {

        if (dpadUp && !previousDpadUp) {
            targetVelocity += 100;
        }

        if (dpadDown && !previousDpadDown) {
            targetVelocity = Math.max(0, targetVelocity - 100);
        }

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;

        // Command both flywheels to the same target velocity.
        motor.setVelocity(targetVelocity);

        double velocity = motor.getVelocity();

        upToSpeed = targetVelocity > 0 && Math.abs(velocity - targetVelocity) < 50;
    }

    public void adjustPIDF(double pChange, double fChange) {
        // Get current coefficients for RUN_USING_ENCODER (or RUN_TO_POSITION)
        PIDFCoefficients pidf = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Modify the values (e.g., Proportional or Feedforward)
        pidf.p += pChange;
        pidf.f += fChange;

        // Prevent negative values
        pidf.p = Math.max(0, pidf.p);
        pidf.f = Math.max(0, pidf.f);

        // Apply them back to the motor
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public PIDFCoefficients getPIDFcoefficients(){
        return motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return (
                motor.getVelocity()
        );
    }

    public double getFlywheelVelocity() {
        return motor.getVelocity();
    }


    public void slowSpeed() {
        targetVelocity = 200;
        motor.setVelocity(targetVelocity);
    }

    /**
     * Stops  flywheel and resets the target velocity.
     */
    public void stop() {
        targetVelocity = 0;
        upToSpeed = false;

        motor.setVelocity(0);
    }

    // --------------------------------------------------
    // LIMELIGHT AUTO-AIM
    // --------------------------------------------------

    /**
     * Toggles auto-aim and approaches the closest visible
     * BIOBUZZ AprilTag.
     *
     * @param togglePressed True when Gamepad 2 A is pressed.
     * @return True when auto-aim is handling the drivetrain.
     */
    public boolean updateAutoAim(boolean togglePressed) {

        if (togglePressed) {
            autoAimEnabled = !autoAimEnabled;

            if (!autoAimEnabled) {
                stopDrive();
                return false;
            }
        }

        if (!autoAimEnabled) {
            return false;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            selectedTagId = -1;
            selectedTagDistance = -1;
            stopDrive();

            telemetry.addLine("Auto-aim: Waiting for Limelight");
            return true;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        LLResultTypes.FiducialResult closestTag = null;
        double closestDistance = Double.POSITIVE_INFINITY;

        // Find the nearest valid BIOBUZZ tag.
        for (LLResultTypes.FiducialResult tag : tags) {

            int id = tag.getFiducialId();

            if (id < MIN_TAG_ID || id > MAX_TAG_ID) {
                continue;
            }

            Pose3D targetPose = tag.getTargetPoseCameraSpace();

            if (targetPose == null) {
                continue;
            }

            Position position =
                    targetPose.getPosition().toUnit(DistanceUnit.METER);

            double distance = Math.sqrt(
                    position.x * position.x
                            + position.y * position.y
                            + position.z * position.z
            );

            if (!Double.isFinite(distance) || distance <= 0) {
                continue;
            }

            if (distance < closestDistance) {
                closestDistance = distance;
                closestTag = tag;
            }
        }

        // No suitable AprilTag found.
        if (closestTag == null) {
            selectedTagId = -1;
            selectedTagDistance = -1;
            stopDrive();

            telemetry.addLine("Auto-aim: No BIOBUZZ tag found");
            return true;
        }

        selectedTagId = closestTag.getFiducialId();
        selectedTagDistance = closestDistance;
        selectedTagAngle = closestTag.getTargetXDegrees();

        double tx = selectedTagAngle;

        // Turn toward the target.
        // Reverse this sign if the robot turns away from the tag.
        double turnPower = -tx * TURN_KP;

        turnPower = clip(
                turnPower,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );

        if (Math.abs(tx) > TURN_TOLERANCE_DEGREES) {

            // Align before moving forward.
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            turnPower
                    )
            );

        } else if (closestDistance > STOP_DISTANCE_METERS) {

            // Approach the tag when approximately aligned.
            double forwardPower =
                    (closestDistance - STOP_DISTANCE_METERS)
                            * APPROACH_KP;

            forwardPower = clip(
                    forwardPower,
                    MIN_FORWARD_POWER,
                    MAX_FORWARD_POWER
            );

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(forwardPower, 0),
                            0
                    )
            );

        } else {

            // Stop at the configured distance.
            stopDrive();
        }

        return true;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public int getSelectedTagId() {
        return selectedTagId;
    }

    public double getSelectedTagDistance() {
        return selectedTagDistance;
    }

    public double getSelectedTagAngle() {
        return selectedTagAngle;
    }

    /**
     * Stops the drivetrain.
     */
    private void stopDrive() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }

    /**
     * Shuts down the Limelight and shooter.
     * Call this once when the OpMode ends.
     */
    public void shutdown() {
        stop();
        autoAimEnabled = false;
        stopDrive();

        if (limelight != null) {
            limelight.stop();
        }
    }

    private double clip(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}