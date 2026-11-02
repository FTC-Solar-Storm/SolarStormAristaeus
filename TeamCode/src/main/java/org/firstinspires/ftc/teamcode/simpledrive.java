package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SimpleDrive")
public class simpledrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        TankDrive drive = new TankDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        Intake intake = new Intake(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap, telemetry, drive);
        ColorVision vision = new ColorVision(hardwareMap);

        boolean previousRightHoodUp = false;
        boolean previousRightHoodDown = false;
        boolean previousLeftHoodUp = false;
        boolean previousLeftHoodDown = false;

        waitForStart();

        while (opModeIsActive()) {

                // DRIVE
                double forward = -gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

                shooter.updateAutoAim(gamepad2.a);

                // COLOR VISION AUTO PICKUP (Mapped to Gamepad 1 Right Bumper)
                if (gamepad1.right_bumper && vision.isDetected()) {

                    double targetAngle = vision.getOffsetAngle();

                    // Drive forward AND steer toward yellow ball simultaneously
                    double forwardPower = Math.abs(targetAngle) > 25.0 ? 0.0 : 0.65;
                    double turnPower = targetAngle * 0.025;

                    if (forwardPower == 0.0) {
                        double magnitude = Math.max(0.25, Math.abs(turnPower));
                        turnPower = Math.copySign(magnitude, turnPower);
                        turnPower = Math.max(-0.6, Math.min(0.6, turnPower));
                    } else {
                        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
                    }

                    // Steer toward ball
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(forwardPower, 0),
                                    -turnPower
                            )
                    );

                    // Run intake while auto-picking up
                    intake.intakeMotor.setPower(1.0);
                    intake.left.setPower(-1.0);
                    intake.right.setPower(1.0);

                } else if (!shooter.isAutoAimEnabled()) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(forward, 0),
                                    turn
                            )
                    );
                }

                // INTAKE AND TRANSFER
                double intakePower = gamepad1.left_trigger;

                if (!gamepad1.right_bumper) {
                    intake.intakeMotor.setPower(intakePower);
                    intake.left.setPower(-intakePower);
                    intake.right.setPower(intakePower);
                }



                // SHOOTER
                // Adjust target velocity anytime via D-pad (even when trigger is released)
                shooter.adjustTargetVelocity(
                        gamepad1.dpad_up,
                        gamepad1.dpad_down
                );

                // Hold Right Trigger to spin flywheels at target velocity
                if (gamepad1.right_trigger > 0.1) {
                    shooter.spin();
                } else {
                    shooter.stop();
                }

                // HOODS
                boolean rightHoodUp = gamepad1.y;
                boolean rightHoodDown = gamepad1.a;
                boolean leftHoodUp = gamepad1.x;
                boolean leftHoodDown = gamepad1.b;

                shooter.adjustRightHood(
                        rightHoodUp && !previousRightHoodUp,
                        rightHoodDown && !previousRightHoodDown
                );

                shooter.adjustLeftHood(
                        leftHoodUp && !previousLeftHoodUp,
                        leftHoodDown && !previousLeftHoodDown
                );

                previousRightHoodUp = rightHoodUp;
                previousRightHoodDown = rightHoodDown;
                previousLeftHoodUp = leftHoodUp;
                previousLeftHoodDown = leftHoodDown;

                // APPLY PIDF VALUES
                // Allows Dashboard coefficient changes to apply
                // even when the shooter trigger is released.
                shooter.applyPIDF();

                // CONTROLS
                telemetry.addLine("========== CONTROLS ==========");
                telemetry.addLine("Left Stick Y: Drive Forward/Backward");
                telemetry.addLine("Right Stick X: Turn");
                telemetry.addLine("Left Trigger: Manual Intake");
                telemetry.addLine("Right Bumper: Color Vision Auto Pickup");
                telemetry.addLine("Left Bumper: Transfer");
                telemetry.addLine("Right Trigger: Enable Shooter");
                telemetry.addLine("D-Pad Up/Down: Adjust Target Velocity");
                telemetry.addLine("Y/A: Right Hood Increase/Decrease");
                telemetry.addLine("X/B: Left Hood Increase/Decrease");
                telemetry.addLine("Gamepad 2 A: Toggle AprilTag Auto-Aim");

                // DRIVE TELEMETRY
                telemetry.addLine("========== DRIVE ==========");
                telemetry.addData("Forward", "%.2f", forward);
                telemetry.addData("Turn", "%.2f", turn);

                // INTAKE TELEMETRY
                telemetry.addLine("========== INTAKE ==========");
                telemetry.addData("Intake Power", "%.2f", intakePower);
                telemetry.addData(
                        "Transfer",
                        gamepad1.left_bumper ? "ON" : "OFF"
                );

                // SHOOTER TELEMETRY
                telemetry.addLine("========== SHOOTER ==========");
                telemetry.addData(
                        "Target Velocity",
                        "%.1f",
                        shooter.getTargetVelocity()
                );
                telemetry.addData(
                        "Right Flywheel Velocity",
                        "%.1f",
                        shooter.getFlywheel1Velocity()
                );
                telemetry.addData(
                        "Left Flywheel Velocity",
                        "%.1f",
                        shooter.getFlywheel2Velocity()
                );
                telemetry.addData(
                        "Average Velocity",
                        "%.1f",
                        shooter.getCurrentVelocity()
                );

                telemetry.addData(
                        "PIDF",
                        "P: %.2f | I: %.2f | D: %.2f | F: %.2f",
                        Shooter.FLYWHEEL_P,
                        Shooter.FLYWHEEL_I,
                        Shooter.FLYWHEEL_D,
                        Shooter.FLYWHEEL_F
                );

                // HOOD TELEMETRY
                telemetry.addLine("========== HOODS ==========");
                telemetry.addData(
                        "Right Hood Position",
                        "%.2f",
                        shooter.getRightHoodPosition()
                );
                telemetry.addData(
                        "Left Hood Position",
                        "%.2f",
                        shooter.getLeftHoodPosition()
                );

                // AUTO-AIM TELEMETRY
                telemetry.addLine("========== AUTO AIM ==========");
                telemetry.addData(
                        "Enabled",
                        shooter.isAutoAimEnabled()
                );
                telemetry.addData(
                        "Selected Tag",
                        shooter.getSelectedTagId()
                );
                telemetry.addData(
                        "Tag Distance",
                        "%.2f",
                        shooter.getSelectedTagDistance()
                );
                telemetry.addData(
                        "Tag Angle",
                        "%.2f",
                        shooter.getSelectedTagAngle()
                );

                telemetry.update();
            }
    }
}
