package org.firstinspires.ftc.teamcode;

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

@Config
public class Shooter {
    // Tune these on the robot; values depend on the motor and hub.
    public static double FLYWHEEL_P = 10.0;
    public static double FLYWHEEL_I = 3.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 12.0;
    public static double VELOCITY_STEP = 50.0;
    public static double HOOD_STEP = 0.1;

    private final DcMotorEx flywheel1, flywheel2;
    private final Servo rightShooterHood, leftShooterHood;
    private final TankDrive drive;

    private double targetVelocity = 50;
    private boolean previousDpadUp, previousDpadDown;

    public enum HiveTarget {
        BLUE_HIVE_TOP(13.0, 13.0, "Blue Hive Top"),
        RED_HIVE_TOP(13.0, -13.0, "Red Hive Top"),
        RED_HIVE_BOTTOM(-13.0, -13.0, "Red Hive Bottom"),
        BLUE_HIVE_BOTTOM(-13.0, 13.0, "Blue Hive Bottom");

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
    private double hiveDistanceInches = 0.0;

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

    public void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // Adjusts target velocity via D-pad buttons without needing motors to be running
    public void adjustTargetVelocity(boolean dpadUp, boolean dpadDown) {
        if (dpadUp && !previousDpadUp) targetVelocity += VELOCITY_STEP;
        if (dpadDown && !previousDpadDown) targetVelocity = Math.max(0, targetVelocity - VELOCITY_STEP);
        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;
    }

    // Call every OpMode loop if driving shooter via D-pad and running simultaneously.
    public void update(boolean dpadUp, boolean dpadDown) {
        adjustTargetVelocity(dpadUp, dpadDown);
        applyPIDF(); // Lets FTC Dashboard PIDF changes take effect without rebuilding.
        setFlywheelVelocity(targetVelocity);
    }

    // Spin flywheels at the currently configured target velocity
    public void spin() {
        setFlywheelVelocity(targetVelocity);
    }

    private void setFlywheelVelocity(double velocity) {
        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }

    // Stops motor motion while preserving targetVelocity setting
    public void stop() {
        setFlywheelVelocity(0);
    }

    public void slowSpeed() {
        targetVelocity = Math.max(0, targetVelocity - VELOCITY_STEP);
        setFlywheelVelocity(targetVelocity);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getFlywheel1Velocity() { return flywheel1.getVelocity(); }
    public double getFlywheel2Velocity() { return flywheel2.getVelocity(); }
    public double getCurrentVelocity() { return (getFlywheel1Velocity() + getFlywheel2Velocity()) / 2.0; }

    public void adjustRightHood(boolean increase, boolean decrease) {
        rightShooterHood.setPosition(adjustPosition(rightShooterHood.getPosition(), increase, decrease));
    }

    public void adjustLeftHood(boolean increase, boolean decrease) {
        leftShooterHood.setPosition(adjustPosition(leftShooterHood.getPosition(), increase, decrease));
    }

    private double adjustPosition(double position, boolean increase, boolean decrease) {
        if (increase != decrease) position += increase ? HOOD_STEP : -HOOD_STEP;
        return clamp(position, 0, 1);
    }

    public double getRightHoodPosition() { return rightShooterHood.getPosition(); }
    public double getLeftHoodPosition() { return leftShooterHood.getPosition(); }

    /**
     * Calculates and returns the heading offset angle in radians to point shooter (BACK) at target Hive.
     * Positive (+) = Turn Left, Negative (-) = Turn Right.
     */
    public double calculateHiveHeadingOffsetRad(HiveTarget target) {
        if (drive == null || target == null) return 0;

        drive.updatePoseEstimate();
        Pose2d currentPose = drive.localizer.getPose();

        double rx = currentPose.position.x;
        double ry = currentPose.position.y;
        double rHeading = currentPose.heading.toDouble(); // radians

        double dx = target.x - rx;
        double dy = target.y - ry;

        hiveDistanceInches = Math.hypot(dx, dy);

        // Field angle to Hive; shooter in BACK = target + 180 deg (PI rad)
        double angleToHiveRad = Math.atan2(dy, dx);
        double targetShooterHeadingRad = angleToHiveRad + Math.PI;

        // Calculate heading offset angle in radians
        double offsetRad = targetShooterHeadingRad - rHeading;
        while (offsetRad > Math.PI) offsetRad -= 2.0 * Math.PI;
        while (offsetRad < -Math.PI) offsetRad += 2.0 * Math.PI;

        lastHeadingOffsetDeg = Math.toDegrees(offsetRad);
        return offsetRad;
    }

    public double calculateHiveHeadingOffsetRad() {
        return calculateHiveHeadingOffsetRad(selectedHiveTarget);
    }

    public void cycleHiveTarget() {
        HiveTarget[] targets = HiveTarget.values();
        int nextOrdinal = (selectedHiveTarget.ordinal() + 1) % targets.length;
        selectedHiveTarget = targets[nextOrdinal];
    }

    public void setHiveTarget(HiveTarget target) {
        if (target != null) {
            this.selectedHiveTarget = target;
        }
    }

    public HiveTarget getSelectedHiveTarget() { return selectedHiveTarget; }
    public double getLastHeadingOffsetDeg() { return lastHeadingOffsetDeg; }
    public double getHiveDistanceInches() { return hiveDistanceInches; }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void shutdown() {
        stop();
        if (drive != null) drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
