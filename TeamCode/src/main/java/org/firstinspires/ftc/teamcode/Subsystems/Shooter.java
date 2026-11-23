package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TankDrive;

@Config
public class Shooter {

    // TUNING VALUES
    public static double FLYWHEEL_P = 16.0;
    public static double FLYWHEEL_I = 3.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 13.5;

    public static double VELOCITY_STEP = 50.0;
    public static double HOOD_STEP = 0.1;

    private final DcMotorEx flywheel1, flywheel2;

    private final Servo rightShooterHood, leftShooterHood;

    private final TankDrive drive;

    // MANUAL TARGET VELOCITY
    private double targetVelocity = 50;

    // SEPARATE AUTOMATIC TARGET VELOCITIES
    private double rightTargetVelocity = 50;
    private double leftTargetVelocity = 50;

    private boolean previousDpadUp, previousDpadDown;

    // AUTOMATIC DISTANCE SHOOTER MODE
    private boolean distanceShooterActive = false;

    // ==========================================
    // HIVE TARGETS
    // ==========================================

    public enum HiveTarget {

        BLUE_HIVE_TOP(15.0, 13.0, "Blue Hive Top"),
        RED_HIVE_TOP(15.0, -13.0, "Red Hive Top"),
        RED_HIVE_BOTTOM(-15.0, -13.0, "Red Hive Bottom"),
        BLUE_HIVE_BOTTOM(-15.0, 13.0, "Blue Hive Bottom");

        public final double x;
        public final double y;
        public final String name;

        HiveTarget(double x, double y, String name) {
            this.x = x;
            this.y = y;
            this.name = name;
        }
    }

    private HiveTarget selectedHiveTarget = HiveTarget.BLUE_HIVE_TOP;

    private double lastHeadingOffsetDeg = 0.0;
    private double hiveDistance = 0.0;

    private boolean isHiveAlignActive = false;
    private long hiveAlignStartTimeMs = 0;
    private static final long HIVE_ALIGN_TIMEOUT_MS = 5000; // 5 seconds auto-timeout

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, TankDrive drive) {

        this.drive = drive;

        flywheel1 = hardwareMap.get(DcMotorEx.class, "rightShooter");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "leftShooter");

        flywheel2.setDirection(DcMotor.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooterHood = hardwareMap.get(Servo.class, "rightShooterHood");
        leftShooterHood = hardwareMap.get(Servo.class, "leftShooterHood");

        rightShooterHood.setPosition(1.0);
        leftShooterHood.setPosition(0.0);

        applyPIDF();
    }

    // ==========================================
    // PIDF
    // ==========================================

    public void applyPIDF() {

        PIDFCoefficients pidf = new PIDFCoefficients(
                FLYWHEEL_P,
                FLYWHEEL_I,
                FLYWHEEL_D,
                FLYWHEEL_F
        );

        flywheel1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        flywheel2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );
    }

    // ==========================================
    // ORIGINAL UPDATE METHODS
    // ==========================================

    public void update() {

        if (drive != null) {

            drive.updatePoseEstimate();

            Pose2d currentPose = drive.localizer.getPose();

            double rx = currentPose.position.x;
            double ry = currentPose.position.y;
            double rHeading = currentPose.heading.toDouble();

            double dx = selectedHiveTarget.x - rx;
            double dy = selectedHiveTarget.y - ry;

            hiveDistance = Math.hypot(dx, dy);

            double angleToHiveRad = Math.atan2(dy, dx);

            // SHOOTER FACES BACKWARD
            double targetShooterHeadingRad = angleToHiveRad + Math.PI;

            double offsetRad = targetShooterHeadingRad - rHeading;

            while (offsetRad > Math.PI) {
                offsetRad -= 2.0 * Math.PI;
            }

            while (offsetRad < -Math.PI) {
                offsetRad += 2.0 * Math.PI;
            }

            lastHeadingOffsetDeg = Math.toDegrees(offsetRad);
        }

        applyPIDF();
    }

    // ORIGINAL OVERLOADED UPDATE METHOD
    public void update(boolean dpadUp, boolean dpadDown) {

        adjustTargetVelocity(dpadUp, dpadDown);

        update();
    }

    // ==========================================
    // MANUAL TARGET VELOCITY
    // ==========================================

    public void adjustTargetVelocity(boolean dpadUp, boolean dpadDown) {

        if (dpadUp && !previousDpadUp) {

            targetVelocity += VELOCITY_STEP;

            // MANUAL SPEED OVERRIDES DISTANCE TARGETS
            distanceShooterActive = false;
        }

        if (dpadDown && !previousDpadDown) {

            targetVelocity = Math.max(0, targetVelocity - VELOCITY_STEP);

            // MANUAL SPEED OVERRIDES DISTANCE TARGETS
            distanceShooterActive = false;
        }

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;
    }

    // ==========================================
    // SHOOTER CONTROL
    // ==========================================

    public void spin() {

        // USE AUTOMATIC TARGETS IF DISTANCE MODE IS ACTIVE
        if (distanceShooterActive) {

            flywheel1.setVelocity(rightTargetVelocity);
            flywheel2.setVelocity(leftTargetVelocity);

        } else {

            // OTHERWISE USE MANUAL TARGET VELOCITY
            setFlywheelVelocity(targetVelocity);
        }
    }

    private void setFlywheelVelocity(double velocity) {

        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }

    public void stop() {

        // STOP MOTORS WITHOUT RESETTING THEIR TARGETS
        setFlywheelVelocity(0);
    }

    public void slowSpeed() {

        targetVelocity = Math.max(0, targetVelocity - VELOCITY_STEP);

        distanceShooterActive = false;

        setFlywheelVelocity(targetVelocity);
    }

    // AUTOMATIC DISTANCE MODE CONTROL
    public void disableDistanceShooterMode() {
        distanceShooterActive = false;
    }

    public boolean isDistanceShooterActive() {
        return distanceShooterActive;
    }

    // ==========================================
    // SHOOTER TELEMETRY
    // ==========================================

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRightTargetVelocity() {
        return distanceShooterActive ? rightTargetVelocity : targetVelocity;
    }

    public double getLeftTargetVelocity() {
        return distanceShooterActive ? leftTargetVelocity : targetVelocity;
    }

    public double getFlywheel1Velocity() {
        return flywheel1.getVelocity();
    }

    public double getFlywheel2Velocity() {
        return flywheel2.getVelocity();
    }

    public double getCurrentVelocity() {

        return (
                getFlywheel1Velocity() +
                        getFlywheel2Velocity()
        ) / 2.0;
    }

    // ==========================================
    // HOOD CONTROL
    // ==========================================

    public void adjustRightHood(boolean increase, boolean decrease) {

        rightShooterHood.setPosition(
                adjustPosition(
                        rightShooterHood.getPosition(),
                        increase,
                        decrease
                )
        );
    }

    public void adjustLeftHood(boolean increase, boolean decrease) {

        leftShooterHood.setPosition(
                adjustPosition(
                        leftShooterHood.getPosition(),
                        increase,
                        decrease
                )
        );
    }

    private double adjustPosition(
            double position,
            boolean increase,
            boolean decrease
    ) {

        if (increase != decrease) {
            position += increase ? HOOD_STEP : -HOOD_STEP;
        }

        return clamp(position, 0, 1);
    }

    public double getRightHoodPosition() {
        return rightShooterHood.getPosition();
    }

    public double getLeftHoodPosition() {
        return leftShooterHood.getPosition();
    }

    // ==========================================
    // HIVE HEADING CALCULATION
    // ==========================================

    public double calculateHiveHeadingOffsetRad(HiveTarget target) {

        if (drive == null || target == null) {
            return 0;
        }

        drive.updatePoseEstimate();

        Pose2d currentPose = drive.localizer.getPose();

        double rx = currentPose.position.x;
        double ry = currentPose.position.y;

        double rHeading = currentPose.heading.toDouble();

        double dx = target.x - rx;
        double dy = target.y - ry;

        hiveDistance = Math.hypot(dx, dy);

        double angleToHiveRad = Math.atan2(dy, dx);

        // SHOOTER FACES BACKWARD
        double targetShooterHeadingRad = angleToHiveRad + Math.PI;

        double offsetRad = targetShooterHeadingRad - rHeading;

        while (offsetRad > Math.PI) {
            offsetRad -= 2.0 * Math.PI;
        }

        while (offsetRad < -Math.PI) {
            offsetRad += 2.0 * Math.PI;
        }

        lastHeadingOffsetDeg = Math.toDegrees(offsetRad);

        return offsetRad;
    }

    public double calculateHiveHeadingOffsetRad() {

        return calculateHiveHeadingOffsetRad(selectedHiveTarget);
    }

    // ==========================================
    // HIVE AUTO ALIGN
    // ==========================================

    public void startHiveAutoAlign() {
        isHiveAlignActive = true;
    }

    public void stopHiveAutoAlign() {

        isHiveAlignActive = false;

        if (drive != null) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );
        }
    }

    /**
     * PROPORTIONAL HIVE AUTO ALIGN WITH 5-SECOND TIMEOUT & TOGGLE OFF.
     * When toggled ON (Button B pressed):
     * 1. Auto-sets shooter flywheel speeds and hood positions from distance.
     * 2. Auto-aligns shooter (back) to target Hive using proportional turning.
     * 3. Automatically disables auto-align after 5 seconds timeout OR when toggled off manually.
     */
    public boolean updateHiveAutoAlign(boolean togglePressed, HiveTarget target) {

        if (togglePressed) {

            isHiveAlignActive = !isHiveAlignActive;

            if (isHiveAlignActive) {

                autoSetShooterFromDistance();

                hiveAlignStartTimeMs = System.currentTimeMillis();
            } else if (drive != null) {

                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0, 0),
                                0
                        )
                );

                return false;
            }
        }

        if (!isHiveAlignActive || drive == null || target == null) {
            return false;
        }

        // 3. Automatically disable auto align after 5 seconds timeout
        if (System.currentTimeMillis() - hiveAlignStartTimeMs > HIVE_ALIGN_TIMEOUT_MS) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );

            isHiveAlignActive = false;

            return false;
        }

        drive.updatePoseEstimate();

        Pose2d currentPose = drive.localizer.getPose();

        double rx = currentPose.position.x;
        double ry = currentPose.position.y;

        double rHeading = currentPose.heading.toDouble();

        double dx = target.x - rx;
        double dy = target.y - ry;

        hiveDistance = Math.hypot(dx, dy);

        double angleToHiveRad = Math.atan2(dy, dx);

        // SHOOTER FACES BACKWARD
        double targetShooterHeadingRad = angleToHiveRad + Math.PI;

        double headingErrorRad = targetShooterHeadingRad - rHeading;

        while (headingErrorRad > Math.PI) {
            headingErrorRad -= 2.0 * Math.PI;
        }

        while (headingErrorRad < -Math.PI) {
            headingErrorRad += 2.0 * Math.PI;
        }

        lastHeadingOffsetDeg = Math.toDegrees(headingErrorRad);

        // STOP WITHIN 2 DEGREE TOLERANCE
        if (Math.abs(lastHeadingOffsetDeg) <= 2.0) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );

            isHiveAlignActive = false;

            return false;
        }

        // ORIGINAL PROPORTIONAL TURNING
        double turnPower = lastHeadingOffsetDeg * 0.02;

        double magnitude = Math.max(0.20, Math.abs(turnPower));

        turnPower = Math.copySign(magnitude, turnPower);

        turnPower = clamp(turnPower, -0.65, 0.65);

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        turnPower
                )
        );

        return true;
    }

    // ORIGINAL OVERLOADED METHOD
    public boolean updateHiveAutoAlign(boolean togglePressed) {

        return updateHiveAutoAlign(
                togglePressed,
                selectedHiveTarget
        );
    }

    public boolean isHiveAlignActive() {
        return isHiveAlignActive;
    }

    public boolean isHiveAligned() {
        return Math.abs(lastHeadingOffsetDeg) <= 2.0;
    }

    // ==========================================
    // HIVE TARGET MANAGEMENT
    // ==========================================

    public void cycleHiveTarget() {

        HiveTarget[] targets = HiveTarget.values();

        int nextOrdinal =
                (selectedHiveTarget.ordinal() + 1)
                        % targets.length;

        selectedHiveTarget = targets[nextOrdinal];
    }

    public void cycleRedHives() {
        selectedHiveTarget = (selectedHiveTarget == HiveTarget.RED_HIVE_TOP) ? HiveTarget.RED_HIVE_BOTTOM : HiveTarget.RED_HIVE_TOP;
    }

    public void cycleBlueHives() {
        selectedHiveTarget = (selectedHiveTarget == HiveTarget.BLUE_HIVE_TOP) ? HiveTarget.BLUE_HIVE_BOTTOM : HiveTarget.BLUE_HIVE_TOP;
    }

    public void setHiveTarget(HiveTarget target) {

        if (target != null) {
            this.selectedHiveTarget = target;
        }
    }

    public HiveTarget getSelectedHiveTarget() {
        return selectedHiveTarget;
    }

    public double getLastHeadingOffsetDeg() {
        return lastHeadingOffsetDeg;
    }

    public double getHiveDistance() {
        return hiveDistance;
    }

    // ==========================================
    // AUTOMATIC SHOOTER SETUP FROM DISTANCE
    // ==========================================

    public void autoSetShooterFromDistance() {

        if (drive != null) {

            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            hiveDistance = Math.hypot(
                    selectedHiveTarget.x - pose.position.x,
                    selectedHiveTarget.y - pose.position.y
            );
        }

        double x = Math.max(1.0, hiveDistance);

        // CALCULATE FLYWHEEL TARGET SPEEDS

        double leftSpeed = 2.09 * x + 1666.0;

        double rightSpeed = 3.24 * x + 1537.0;

        // CALCULATE HOOD POSITIONS

        double leftHoodPos = clamp(
                1.71 + (-0.245 * Math.log(x)),
                0.0,
                1.0
        );

        double rightHoodPos = clamp(
                -0.612 + (0.0382 * x) + (-3.48e-4 * x * x),
                0.0,
                1.0
        );


        rightTargetVelocity = rightSpeed;
        leftTargetVelocity = leftSpeed;

        // ENABLE DISTANCE TARGET MODE
        distanceShooterActive = true;

        // MOVE HOODS AUTOMATICALLY

        leftShooterHood.setPosition(leftHoodPos);
        rightShooterHood.setPosition(rightHoodPos);
    }

    public void autoSetRPMFromDistance() {

        autoSetShooterFromDistance();
    }

    // ==========================================
    // UTILITY
    // ==========================================

    private double clamp(double value, double min, double max) {

        return Math.max(min, Math.min(max, value));
    }

    // ==========================================
    // SHUTDOWN
    // ==========================================

    public void shutdown() {

        stop();

        if (drive != null) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );
        }
    }
}