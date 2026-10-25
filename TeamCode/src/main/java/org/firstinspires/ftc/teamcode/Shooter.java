package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@Config
public class Shooter {

    // ==========================================
    // FTC DASHBOARD: LIVE PIDF CONFIGURATION
    // ==========================================

    // Replace these placeholders with suitable starting values
    // for your motor/controller before running the flywheels.
    public static double FLYWHEEL_P = 0.0;
    public static double FLYWHEEL_I = 0.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 0.0;

    // Minimum change required before PIDF is reapplied.
    private static final double PIDF_EPSILON = 0.000001;

    // ==========================================
    // HARDWARE
    // ==========================================

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    private final Servo rightShooterHood;
    private final Servo leftShooterHood;

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final TankDrive drive;

    // ==========================================
    // SHOOTER STATE
    // ==========================================

    private double targetVelocity = 0.0;

    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;

    // ==========================================
    // HOOD CONFIGURATION
    // ==========================================

    public static double HOOD_STEP = 0.1;

    private static final double MIN_SERVO_POSITION = 0.0;
    private static final double MAX_SERVO_POSITION = 1.0;

    // ==========================================
    // AUTO-AIM STATE
    // ==========================================

    private boolean autoAimEnabled = false;
    private boolean previousAutoAimToggle = false;

    private int selectedTagId = -1;
    private double selectedTagDistance = -1.0;
    private double selectedTagAngle = 0.0;

    private static final double AUTO_AIM_TURN_GAIN = 0.02;
    private static final double AUTO_AIM_MAX_TURN = 0.35;
    private static final double AUTO_AIM_TOLERANCE_DEGREES = 1.0;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public Shooter(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            TankDrive drive
    ) {
        this.telemetry = telemetry;
        this.drive = drive;

        // Shooter motors
        flywheel1 = hardwareMap.get(
                DcMotorEx.class,
                "rightShooter"
        );

        flywheel2 = hardwareMap.get(
                DcMotorEx.class,
                "leftShooter"
        );

        // Velocity control
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Hood servos
        rightShooterHood = hardwareMap.get(
                Servo.class,
                "rightShooterHood"
        );

        leftShooterHood = hardwareMap.get(
                Servo.class,
                "leftShooterHood"
        );

        rightShooterHood.setPosition(1.0);
        leftShooterHood.setPosition(0.0);

        // Limelight
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.start();

        // Apply the initial Dashboard PIDF values.
        applyPIDFIfChanged();
    }

    // ==========================================
    // LIVE PIDF TUNING
    // ==========================================

    /**
     * Applies PIDF values only when they change in FTC Dashboard.
     * The motor controller receives the new coefficients without
     * repeatedly writing the same values every loop.
     */
    public void applyPIDFIfChanged() {

        if (Double.isNaN(FLYWHEEL_P)
                || Double.isNaN(FLYWHEEL_I)
                || Double.isNaN(FLYWHEEL_D)
                || Double.isNaN(FLYWHEEL_F)
                || Double.isInfinite(FLYWHEEL_P)
                || Double.isInfinite(FLYWHEEL_I)
                || Double.isInfinite(FLYWHEEL_D)
                || Double.isInfinite(FLYWHEEL_F)) {

            telemetry.addLine("Invalid PIDF values; update rejected.");
            return;
        }

        boolean changed =
                Math.abs(FLYWHEEL_P - lastP) > PIDF_EPSILON
                        || Math.abs(FLYWHEEL_I - lastI) > PIDF_EPSILON
                        || Math.abs(FLYWHEEL_D - lastD) > PIDF_EPSILON
                        || Math.abs(FLYWHEEL_F - lastF) > PIDF_EPSILON;

        if (!changed) {
            return;
        }

        PIDFCoefficients pidf = new PIDFCoefficients(
                FLYWHEEL_P,
                FLYWHEEL_I,
                FLYWHEEL_D,
                FLYWHEEL_F
        );

        try {
            flywheel1.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    pidf
            );

            flywheel2.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    pidf
            );

            lastP = FLYWHEEL_P;
            lastI = FLYWHEEL_I;
            lastD = FLYWHEEL_D;
            lastF = FLYWHEEL_F;

            telemetry.addLine("Shooter PIDF updated.");

        } catch (UnsupportedOperationException e) {
            telemetry.addLine(
                    "This motor controller does not support PIDF updates."
            );
        }
    }

    // ==========================================
    // SHOOTER VELOCITY CONTROL
    // ==========================================

    /**
     * Adjusts target velocity by 50 ticks/sec per button press.
     * Call once per OpMode loop.
     */
    public void update(boolean dpadUp, boolean dpadDown) {

        if (dpadUp && !previousDpadUp) {
            targetVelocity += 50.0;
        }

        if (dpadDown && !previousDpadDown) {
            targetVelocity -= 50.0;
        }

        targetVelocity = Math.max(0.0, targetVelocity);

        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;

        flywheel1.setVelocity(targetVelocity);
        flywheel2.setVelocity(targetVelocity);

        // Detect Dashboard changes while the shooter is running.
        applyPIDFIfChanged();
    }

    public void stop() {
        targetVelocity = 0.0;

        flywheel1.setVelocity(0.0);
        flywheel2.setVelocity(0.0);
    }

    public void slowSpeed() {
        targetVelocity = Math.max(0.0, targetVelocity - 50.0);

        flywheel1.setVelocity(targetVelocity);
        flywheel2.setVelocity(targetVelocity);
    }

    // ==========================================
    // VELOCITY GETTERS
    // ==========================================

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getFlywheel1Velocity() {
        return flywheel1.getVelocity();
    }

    public double getFlywheel2Velocity() {
        return flywheel2.getVelocity();
    }

    public double getCurrentVelocity() {
        return (
                flywheel1.getVelocity()
                        + flywheel2.getVelocity()
        ) / 2.0;
    }

    // ==========================================
    // HOOD CONTROLS
    // ==========================================

    public void adjustRightHood(
            boolean increase,
            boolean decrease
    ) {
        double position = rightShooterHood.getPosition();

        if (increase) {
            position += HOOD_STEP;
        }

        if (decrease) {
            position -= HOOD_STEP;
        }

        position = clamp(
                position,
                MIN_SERVO_POSITION,
                MAX_SERVO_POSITION
        );

        rightShooterHood.setPosition(position);
    }

    public void adjustLeftHood(
            boolean increase,
            boolean decrease
    ) {
        double position = leftShooterHood.getPosition();

        if (increase) {
            position += HOOD_STEP;
        }

        if (decrease) {
            position -= HOOD_STEP;
        }

        position = clamp(
                position,
                MIN_SERVO_POSITION,
                MAX_SERVO_POSITION
        );

        leftShooterHood.setPosition(position);
    }

    public double getRightHoodPosition() {
        return rightShooterHood.getPosition();
    }

    public double getLeftHoodPosition() {
        return leftShooterHood.getPosition();
    }

    // ==========================================
    // LIMELIGHT AUTO-AIM
    // ==========================================

    public void updateAutoAim(boolean togglePressed) {

        if (togglePressed && !previousAutoAimToggle) {
            autoAimEnabled = !autoAimEnabled;
        }

        previousAutoAimToggle = togglePressed;

        if (!autoAimEnabled) {
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            selectedTagId = -1;
            selectedTagDistance = -1.0;
            selectedTagAngle = 0.0;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        selectedTagId = -1;
        selectedTagDistance = -1.0;
        selectedTagAngle = 0.0;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();

            if (id >= 30 && id <= 45) {

                Position position =
                        fiducial.getTargetPoseCameraSpace().getPosition();

                selectedTagDistance = Math.sqrt(
                        position.x * position.x
                                + position.y * position.y
                                + position.z * position.z
                );

                selectedTagAngle = fiducial.getTargetXDegrees();
                selectedTagId = id;

                break;
            }
        }

        if (selectedTagId != -1) {

            double turn = 0.0;

            if (Math.abs(selectedTagAngle)
                    > AUTO_AIM_TOLERANCE_DEGREES) {

                turn = clamp(
                        selectedTagAngle * AUTO_AIM_TURN_GAIN,
                        -AUTO_AIM_MAX_TURN,
                        AUTO_AIM_MAX_TURN
                );
            }

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0.0, 0.0),
                            turn
                    )
            );
        }
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

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    // ==========================================
    // UTILITIES
    // ==========================================

    private double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }

    // ==========================================
    // SHUTDOWN
    // ==========================================

    public void shutdown() {
        stop();

        limelight.stop();

        if (drive != null) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0.0, 0.0),
                            0.0
                    )
            );
        }
    }
}