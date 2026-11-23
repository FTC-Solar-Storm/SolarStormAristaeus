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
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name = "SimpleDrive")
public class simpledrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        Intake intake = new Intake(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap, telemetry, drive);
        ColorVision vision = new ColorVision(hardwareMap);

        waitForStart();

        // Reset starting position to (0, 0, 0)
        drive.localizer.setPose(new Pose2d(0, 0, 0));
        if (drive.localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) drive.localizer).driver.resetPosAndIMU();
        }

        while (opModeIsActive()) {

            // Central Subsystem Update (Pose, Distance, Target Velocity Steps)
            shooter.update(gamepad1.dpad_up || gamepad2.dpad_up, gamepad1.dpad_down || gamepad2.dpad_down);

            // Drive Inputs (Gamepad 1)
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            boolean isDriving = Math.abs(forward) > 0.05 || Math.abs(turn) > 0.05;

            // Hive Auto-Align Trigger (Gamepad 2 B or Gamepad 1 B)
            // Auto-sets RPM and hoods from distance immediately when auto-align starts
            boolean alignTriggered = gamepad2.bWasPressed() || gamepad1.bWasPressed();
            if (alignTriggered) {
                shooter.autoSetShooterFromDistance();
            }
            boolean hiveAlignActive = shooter.updateHiveAutoAlign(alignTriggered);

            // Target Hive Selection & Auto Shooter Distance Config (Gamepad 2)
            if (gamepad2.xWasPressed()) shooter.cycleHiveTarget();
            if (gamepad2.aWasPressed()) shooter.autoSetShooterFromDistance();

            // Color Vision Auto Pickup (Gamepad 1 Right Bumper or Gamepad 2 Left Bumper)
            boolean autoPickupActive = vision.updateAutoPickup(gamepad1.rightBumperWasPressed() || gamepad2.leftBumperWasPressed(), drive, intake);

            // Manual Drive
            if (!autoPickupActive && !hiveAlignActive) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), turn));
            }

            // Robot motion check (driving, auto-aligning, or auto-pickup)
            boolean isRobotMoving = isDriving || hiveAlignActive || autoPickupActive;

            // Manual Gate Toggle (Gamepad 2 ONLY)
            if (gamepad2.rightBumperWasPressed()) {
                intake.toggleGate();
            }

            // Auto-close gate when robot is moving (does NOT automatically reopen when stopped)
            if (isRobotMoving && intake.isGateOpen) {
                intake.closeGate();
            }

            // Intake & Outtake Controls (Gamepad 1 Triggers)
            if (!vision.isAutoPickupActive()) {
                double intakeVal = gamepad1.right_trigger;   // Right Trigger: Intake
                double outtakeVal = gamepad1.left_trigger;    // Left Trigger: Outtake
                double netPower = (intakeVal - outtakeVal) * 0.8;

                intake.intakeMotor.setPower(netPower);
                intake.left.setPower(-netPower);
                intake.right.setPower(netPower);
            }

            // Shooter Flywheels (Gamepad 2 ONLY)
            boolean spinShooter = gamepad2.right_trigger > 0.1;
            if (spinShooter) {
                shooter.spin();
            } else {
                shooter.stop();
            }

            // Flower & Nectar Controls (Gamepad 1)
            intake.moveFlower(gamepad1.leftBumperWasPressed());
            intake.moveNectar(gamepad1.yWasPressed());

            // Manual Hood Adjustments (Gamepad 2 D-Pad)
            shooter.adjustRightHood(gamepad2.yWasPressed(), gamepad2.aWasPressed());
            shooter.adjustLeftHood(gamepad2.dpadRightWasPressed(), gamepad2.dpadLeftWasPressed());

            // Apply PIDF
            shooter.applyPIDF();

            // ==========================================
            // TELEMETRY
            // ==========================================

            Pose2d currentPose = drive.localizer.getPose();

            telemetry.addLine("========== ROBOT ODOM POSE ==========");
            telemetry.addData("Pose X (in)", "%.2f", currentPose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", currentPose.position.y);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));

            telemetry.addLine("\n========== CONTROLS ==========");
            telemetry.addLine("GP1 Left Stick Y / Right Stick X: Drive");
            telemetry.addLine("GP1 Right Trigger: Intake");
            telemetry.addLine("GP1 Left Trigger: Outtake");
            telemetry.addLine("GP1 Right Bumper: Color Vision Auto Pickup");
            telemetry.addLine("GP1 Left Bumper: Toggle Flower");
            telemetry.addLine("GP1 Y: Toggle Nectar Gate");
            telemetry.addLine("GP1 B / GP2 B: Auto Align Hive & Auto Set RPM");
            telemetry.addLine("GP2 RT: Spin Flywheels (Shoot)");
            telemetry.addLine("GP2 RB: Toggle Gate (Auto-closes on move)");
            telemetry.addLine("GP2 X: Cycle Target Hive");
            telemetry.addLine("GP2 A: Auto Set RPM from Distance");
            telemetry.addLine("GP2 D-Pad: Hood Adjustments");

            telemetry.addLine("\n========== STATUS ==========");
            telemetry.addData("Gate Open", intake.isGateOpen ? "YES" : "NO");
            telemetry.addData("Robot Moving", isRobotMoving ? "YES" : "NO");
            telemetry.addData("Right Flywheel Vel", "%.1f", shooter.getFlywheel1Velocity());
            telemetry.addData("Left Flywheel Vel", "%.1f", shooter.getFlywheel2Velocity());
            telemetry.addData("Right Hood Pos", "%.2f", shooter.getRightHoodPosition());
            telemetry.addData("Left Hood Pos", "%.2f", shooter.getLeftHoodPosition());

            telemetry.addLine("\n========== HIVE AUTO ALIGN ==========");
            telemetry.addData("Target Hive", shooter.getSelectedHiveTarget().name + " (" + shooter.getSelectedHiveTarget().x + ", " + shooter.getSelectedHiveTarget().y + ")");
            telemetry.addData("Target Distance", "%.2f", shooter.getHiveDistance());
            telemetry.addData("Heading Offset", "%.2f deg", shooter.getLastHeadingOffsetDeg());
            telemetry.addData("Aligned (±2.0°)", shooter.isHiveAligned() ? "YES" : "NO");
            telemetry.addData("Auto Align Active", shooter.isHiveAlignActive() ? "YES" : "NO");

            telemetry.update();
        }

        shooter.shutdown();
        vision.stop();
    }
}
