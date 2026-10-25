package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter Test", group = "Test")
public class shootertest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Send telemetry to both the Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // Create the drive object required by your current Shooter constructor
        TankDrive drive = new TankDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        // Initialize shooter using your existing Shooter class
        Shooter shooter = new Shooter(hardwareMap, telemetry, drive);

        // Button state tracking for single-press controls
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;

        telemetry.addLine("Shooter Test Ready");
        telemetry.addLine("Right Trigger: Run shooter");
        telemetry.addLine("D-pad Up/Down: Adjust target velocity");
        telemetry.addLine("A/B: Adjust right hood");
        telemetry.addLine("X/Y: Adjust left hood");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Read controls
                boolean dpadUp = gamepad1.dpad_up;
                boolean dpadDown = gamepad1.dpad_down;

                boolean a = gamepad1.a;
                boolean b = gamepad1.b;
                boolean x = gamepad1.x;
                boolean y = gamepad1.y;

                boolean shooterRunning = gamepad1.right_trigger > 0.1;

                // Adjust target velocity by 50 ticks/sec per button press
                shooter.update(
                        dpadUp && !previousDpadUp,
                        dpadDown && !previousDpadDown
                );

                // Run shooter while right trigger is held
                if (shooterRunning) {
                    // shooter.update() already controls the flywheel target velocity
                } else {
                    shooter.stop();
                }

                // Adjust right hood
                shooter.adjustRightHood(
                        a && !previousA,
                        b && !previousB
                );

                // Adjust left hood
                shooter.adjustLeftHood(
                        x && !previousX,
                        y && !previousY
                );

                // Save button states
                previousDpadUp = dpadUp;
                previousDpadDown = dpadDown;

                previousA = a;
                previousB = b;
                previousX = x;
                previousY = y;

                // ==========================================
                // FTC DASHBOARD GRAPHABLE TELEMETRY
                // ==========================================

                telemetry.addData(
                        "Shooter/Target Velocity",
                        shooter.getTargetVelocity()
                );

                telemetry.addData(
                        "Shooter/Right Flywheel Velocity",
                        shooter.getFlywheel1Velocity()
                );

                telemetry.addData(
                        "Shooter/Left Flywheel Velocity",
                        shooter.getFlywheel2Velocity()
                );

                telemetry.addData(
                        "Shooter/Right Hood Position",
                        shooter.getRightHoodPosition()
                );

                telemetry.addData(
                        "Shooter/Left Hood Position",
                        shooter.getLeftHoodPosition()
                );

                telemetry.addData(
                        "Shooter/Running",
                        shooterRunning
                );

                telemetry.update();
            }

        } finally {
            shooter.shutdown();
        }
    }
}