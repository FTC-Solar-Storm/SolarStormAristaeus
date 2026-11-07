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

        // CONNECT TELEMETRY TO FTC DASHBOARD
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // INITIALIZE DRIVE
        TankDrive drive = new TankDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        // INITIALIZE SUBSYSTEMS
        Intake intake = new Intake(hardwareMap, telemetry);

        Shooter shooter = new Shooter(
                hardwareMap,
                telemetry,
                drive
        );

        // INITIALIZE LOGITECH C920 CAMERA
        ColorVision vision = new ColorVision(hardwareMap);

        // HOOD BUTTON STATES
        boolean previousRightHoodUp = false;
        boolean previousRightHoodDown = false;
        boolean previousLeftHoodUp = false;
        boolean previousLeftHoodDown = false;

        // TOGGLE BUTTON STATES
        boolean previousAutoPickupButton = false;
        boolean previousGateButton = false;

        waitForStart();

        while (opModeIsActive()) {

            // DRIVE
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            // APRILTAG AUTO AIM
            shooter.updateAutoAim(gamepad2.a);

            // COLOR VISION AUTO PICKUP
            boolean autoPickupButton = gamepad1.right_bumper;

            boolean autoPickupPressed =
                    autoPickupButton && !previousAutoPickupButton;

            previousAutoPickupButton = autoPickupButton;

            boolean autoPickupRunning = vision.updateAutoPickup(
                    autoPickupPressed,
                    drive,
                    intake
            );

            // MANUAL DRIVE
            if (!autoPickupRunning && !shooter.isAutoAimEnabled()) {

                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(forward, 0),
                                turn
                        )
                );
            }

            // GATE TOGGLE


            intake.moveGate(gamepad2.rightBumperWasPressed());




            // MANUAL INTAKE
            double intakePower = gamepad1.left_trigger*0.6;

            if (!vision.isAutoPickupActive()) {

                intake.intakeMotor.setPower(intakePower);
                intake.left.setPower(-intakePower);
                intake.right.setPower(intakePower);
            }

            // SHOOTER
            shooter.adjustTargetVelocity(
                    gamepad1.dpad_up,
                    gamepad1.dpad_down
            );

            // HOLD RIGHT TRIGGER TO SPIN FLYWHEELS
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
            shooter.applyPIDF();
            Pose2d currentPose = drive.localizer.getPose();
            telemetry.addData("Pose X (in)", "%.2f", currentPose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", currentPose.position.y);
            telemetry.addData("Pose Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));
            // ==========================================
            // CONTROLS
            // ==========================================

            telemetry.addLine("========== CONTROLS ==========");
            telemetry.addLine("Left Stick Y: Drive Forward/Backward");
            telemetry.addLine("Right Stick X: Turn");
            telemetry.addLine("Left Trigger: Manual Intake");
            telemetry.addLine("Right Bumper: Toggle Color Vision Auto Pickup");
            telemetry.addLine("Left Bumper: Transfer");
            telemetry.addLine("Right Trigger: Enable Shooter");
            telemetry.addLine("D-Pad Up/Down: Adjust Target Velocity");
            telemetry.addLine("Y/A: Right Hood Increase/Decrease");
            telemetry.addLine("X/B: Left Hood Increase/Decrease");
            telemetry.addLine("Gamepad 2 A: Toggle AprilTag Auto-Aim");
            telemetry.addLine("Gamepad 2 Right Bumper: Toggle Gate");

            // ==========================================
            // DRIVE TELEMETRY
            // ==========================================

            telemetry.addLine("========== DRIVE ==========");

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Turn", "%.2f", turn);

            // ==========================================
            // CAMERA TELEMETRY
            // ==========================================

            telemetry.addLine("========== COLOR VISION ==========");

            telemetry.addData(
                    "Camera Open",
                    vision.isCameraOpened()
            );

            telemetry.addData(
                    "Camera Frames",
                    vision.getFrameCount()
            );

            telemetry.addData(
                    "Object Detected",
                    vision.isDetected()
            );

            telemetry.addData(
                    "Center X",
                    vision.getCenterX()
            );

            telemetry.addData(
                    "Center Y",
                    vision.getCenterY()
            );

            telemetry.addData(
                    "Object Width",
                    vision.getWidth()
            );

            telemetry.addData(
                    "Object Height",
                    vision.getHeight()
            );

            telemetry.addData(
                    "Target Angle",
                    "%.2f degrees",
                    vision.getOffsetAngle()
            );

            telemetry.addData(
                    "Auto Pickup",
                    vision.isAutoPickupActive() ? "ON" : "OFF"
            );

            // ==========================================
            // INTAKE TELEMETRY
            // ==========================================

            telemetry.addLine("========== INTAKE ==========");

            telemetry.addData(
                    "Intake Power",
                    "%.2f",
                    intakePower
            );

            telemetry.addData(
                    "Transfer",
                    gamepad1.left_bumper ? "ON" : "OFF"
            );

            // ==========================================
            // SHOOTER TELEMETRY
            // ==========================================

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

            // ==========================================
            // HOOD TELEMETRY
            // ==========================================

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

            // ==========================================
            // APRILTAG AUTO-AIM TELEMETRY
            // ==========================================

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

        // ==========================================
        // SHUTDOWN
        // ==========================================

        shooter.shutdown();
        vision.stop();
    }
}