package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hive Auto Align", group = "TeleOp")
public class HiveAutoAlign extends LinearOpMode {

    @Override
    public void runOpMode() {

        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        Intake intake = new Intake(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap, telemetry, drive);

        telemetry.addLine("Hive Auto Align TeleOp Initialized.");
        telemetry.update();

        waitForStart();

        // Reset starting position to (0, 0, 0)
        drive.localizer.setPose(new Pose2d(0, 0, 0));
        if (drive.localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) drive.localizer).driver.resetPosAndIMU();
        }

        while (opModeIsActive()) {

            // Single press B calculates offset and runs Actions.runBlocking in TeleOp
            if (gamepad1.bWasPressed()) {
                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();
                double offsetRad = shooter.calculateHiveHeadingOffsetRad();

                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                                .turn(-offsetRad)
                                .build()
                );
            }

            // Single press X cycles through target Hives
            if (gamepad1.xWasPressed()) {
                shooter.cycleHiveTarget();
            }

            // Manual Drive
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(forward, 0),
                            turn
                    )
            );

            // Intake Controls
            double intakePower = gamepad1.left_trigger;
            intake.intakeMotor.setPower(intakePower);
            intake.left.setPower(-intakePower);
            intake.right.setPower(intakePower);

            // Shooter Controls
            shooter.adjustTargetVelocity(gamepad1.dpad_up, gamepad1.dpad_down);
            if (gamepad1.right_trigger > 0.1) {
                shooter.spin();
            } else {
                shooter.stop();
            }

            // Telemetry Display
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

// Continuously update distance and heading offset to selected Hive
            double headingOffsetRad = shooter.calculateHiveHeadingOffsetRad();
            double headingOffsetDeg = Math.toDegrees(headingOffsetRad);

            telemetry.addLine("========== ROBOT ODOM POSE ==========");
            telemetry.addData("Pose X (in)", "%.2f", pose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", pose.position.y);
            telemetry.addData("Heading (deg)", "%.2f",
                    Math.toDegrees(pose.heading.toDouble()));

            telemetry.addLine("\n========== HIVE AUTO ALIGN ==========");
            telemetry.addData("Selected Target",
                    shooter.getSelectedHiveTarget().name);
            telemetry.addData("Target X", "%.2f", shooter.getSelectedHiveTarget().x);
            telemetry.addData("Target Y", "%.2f", shooter.getSelectedHiveTarget().y);

// Distance to selected Hive
            telemetry.addData("Distance to Hive", "%.2f in",
                    shooter.getHiveDistanceInches());

// Continuously updated heading offset
            telemetry.addData("Heading Offset", "%.2f deg",
                    headingOffsetDeg);
            telemetry.addData("Heading Offset (rad)", "%.4f",
                    headingOffsetRad);

            telemetry.addLine("\n========== CONTROLS ==========");
            telemetry.addLine("Gamepad 1 B : Turn to align to Hive");
            telemetry.addLine("Gamepad 1 X : Cycle Target Hive");
            telemetry.addLine("Left Stick Y / Right Stick X : Manual Drive");
            telemetry.addLine("Right Trigger : Shooter");

            telemetry.update();
        }

        shooter.stop();
    }
}
