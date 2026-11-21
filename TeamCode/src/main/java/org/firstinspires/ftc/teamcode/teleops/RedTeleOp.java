package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ColorVision;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        Intake.TankDrive drive = new Intake.TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        Intake intake = new Intake(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap, telemetry, drive);
        ColorVision vision = new ColorVision(hardwareMap);

        // Default to Red Alliance Hive
        shooter.setHiveTarget(Shooter.HiveTarget.RED_HIVE_TOP);

        waitForStart();

        // Reset starting pose to (0, 0, 0)
        drive.localizer.setPose(new Pose2d(0, 0, 0));
        if (drive.localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) drive.localizer).driver.resetPosAndIMU();
        }

        while (opModeIsActive()) {

            // Update shooter pose, distance & offset
            shooter.update(gamepad1.dpad_up, gamepad1.dpad_down);

            // Drive Sticks
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            // Red Hive Auto-Align (Button B = Auto-set shooter & align with 5s timeout / toggle off)
            boolean hiveAlignActive = shooter.updateHiveAutoAlign(gamepad2.bWasPressed() || gamepad1.bWasPressed());

            // Cycle Red Hives ONLY (Red Hive Top <-> Red Hive Bottom)
            if (gamepad2.xWasPressed() || gamepad1.xWasPressed()) {
                shooter.cycleRedHives();
            }

            // Auto Shooter Distance Adjustment
            if (gamepad2.aWasPressed() || gamepad1.aWasPressed()) {
                shooter.autoSetShooterFromDistance();
            }

            // Color Vision Auto Pickup
            boolean autoPickupActive = vision.updateAutoPickup(gamepad1.rightBumperWasPressed(), drive, intake);

            // Manual Drive
            if (!autoPickupActive && !hiveAlignActive) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), turn));
            }

            // Gate Toggle
            intake.moveGate(gamepad2.rightBumperWasPressed());

            // Manual Intake Motor
            double intakePower = gamepad1.left_trigger * 0.6;
            if (!vision.isAutoPickupActive()) {
                intake.intakeMotor.setPower(intakePower);
                intake.left.setPower(-intakePower);
                intake.right.setPower(intakePower);
            }

            // Shooter Flywheels
            if (gamepad1.right_trigger > 0.1) {
                shooter.spin();
            } else {
                shooter.stop();
            }

            // Hood Adjustments
            shooter.adjustRightHood(gamepad1.yWasPressed(), gamepad1.aWasPressed());
            shooter.adjustLeftHood(gamepad1.xWasPressed(), gamepad1.bWasPressed());

            shooter.applyPIDF();

            // Telemetry
            Pose2d currentPose = drive.localizer.getPose();

            telemetry.addLine("========== RED TELEOP ==========");
            telemetry.addData("Pose X (in)", "%.2f", currentPose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", currentPose.position.y);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));

            telemetry.addLine("\n========== RED HIVE AUTO ALIGN ==========");
            telemetry.addData("Selected Hive Target", shooter.getSelectedHiveTarget().name);
            telemetry.addData("Target Distance", "%.2f", shooter.getHiveDistance());
            telemetry.addData("Heading Offset", "%.2f deg", shooter.getLastHeadingOffsetDeg());
            telemetry.addData("Aligned (±2.0°)", shooter.isHiveAligned() ? "YES" : "NO");

            telemetry.update();
        }

        shooter.shutdown();
        vision.stop();
    }
}
