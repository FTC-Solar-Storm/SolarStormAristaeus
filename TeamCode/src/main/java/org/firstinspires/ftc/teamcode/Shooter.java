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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
    private final Limelight3A limelight;
    private final TankDrive drive;

    private double targetVelocity = 50;
    private boolean previousDpadUp, previousDpadDown;
    private boolean autoAimEnabled, previousAutoAimToggle;
    private int selectedTagId = -1;
    private double selectedTagDistance = -1.0, selectedTagAngle;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
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

    public void updateAutoAim(boolean togglePressed) {
        if (togglePressed && !previousAutoAimToggle) {
            autoAimEnabled = !autoAimEnabled;
        }
        previousAutoAimToggle = togglePressed;

        if (!autoAimEnabled) return;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();

                // Only target BioBuzz season AprilTags (IDs 30 through 45)
                if (id < 30 || id > 45) continue;

                selectedTagId = id;
                selectedTagAngle = tag.getTargetXDegrees();

                Position p = tag.getTargetPoseCameraSpace().getPosition();
                if (p != null) {
                    selectedTagDistance = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                } else {
                    selectedTagDistance = -1.0;
                }

                // Turn until within 3 degrees of the AprilTag
                if (Math.abs(selectedTagAngle) > 3.0) {
                    double turnPower = selectedTagAngle * 0.02;

                    turnPower = clamp(turnPower, -0.5, 0.5);

                    drive.setDrivePowers(
                            new PoseVelocity2d(new Vector2d(0, 0), turnPower)
                    );
                } else {
                    drive.setDrivePowers(
                            new PoseVelocity2d(new Vector2d(0, 0), 0)
                    );
                }

                return;
            }
        }

        // No valid AprilTag found
        selectedTagId = -1;
        selectedTagDistance = -1.0;
        selectedTagAngle = 0.0;

        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(0, 0), 0)
        );
    }

    public int getSelectedTagId() { return selectedTagId; }
    public double getSelectedTagDistance() { return selectedTagDistance; }
    public double getSelectedTagAngle() { return selectedTagAngle; }
    public boolean isAutoAimEnabled() { return autoAimEnabled; }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void shutdown() {
        stop();
        limelight.stop();
        if (drive != null) drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
