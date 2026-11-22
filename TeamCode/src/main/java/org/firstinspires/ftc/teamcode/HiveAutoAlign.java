package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
        boolean hiveAlignActive= false;
        while (opModeIsActive()) {

            // Central update loop: updates pose estimate, distance, and heading offset continuously
            //shooter.update(gamepad1.dpad_up, gamepad1.dpad_down);

            // B Button toggles auto-align ON/OFF (pressing B again exits immediately)
            //boolean hiveAlignActive = shooter.updateHiveAutoAlign(gamepad1.bWasPressed());

            // Single press X cycles through target Hives
            if (gamepad1.xWasPressed()) {
                shooter.cycleHiveTarget();
            }
            if (gamepad1.bWasPressed()){
                shooter.hiveAutoAlign();
            }
            // Single press A sets shooter RPM automatically based on distance to selected Hive
            if (gamepad1.aWasPressed()) {
                shooter.autoSetRPMFromDistance();
            }

            // Manual Drive when Auto Align is NOT active
            if (!hiveAlignActive) {
                double forward = -gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(forward, 0),
                                turn
                        )
                );
            }

            // Intake Controls
            double intakePower = gamepad1.left_trigger;
            intake.intakeMotor.setPower(intakePower);
            intake.left.setPower(-intakePower);
            intake.right.setPower(intakePower);

            // Shooter Spin Controls
            if (gamepad1.right_trigger > 0.1) {
                shooter.spin();
            } else {
                shooter.stop();
            }

            // Telemetry Display
            Pose2d pose = drive.localizer.getPose();

            telemetry.addLine("========== ROBOT ODOM POSE ==========");
            telemetry.addData("Pose X (in)", "%.2f", pose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", pose.position.y);
            telemetry.addData("Heading (deg)", "%.2f",
                    Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("TrackingL value is :", shooter.trackingL);
            telemetry.addData("Tracking Point X : ", shooter.trackPosX);
            telemetry.addData("Tracking Point Y : ", shooter.trackPosY);
            telemetry.addLine("\n========== HIVE AUTO ALIGN ==========");
            telemetry.addData("Selected Target",
                    shooter.getSelectedHiveTarget().name);
            telemetry.addData("Target X", "%.2f", shooter.getSelectedHiveTarget().x);
            telemetry.addData("Target Y", "%.2f", shooter.getSelectedHiveTarget().y);

            // Distance to selected Hive
            telemetry.addData("Distance to Hive", "%.2f",
                    shooter.getHiveDistance());

            // Continuously updated heading offset
            telemetry.addData("Heading Offset", "%.2f deg",
                    shooter.getLastHeadingOffsetDeg());
            telemetry.addData("Aligned (±2.0°)",
                    shooter.isHiveAligned() ? "YES" : "NO");

            telemetry.addLine("\n========== CONTROLS ==========");
            telemetry.addLine("Gamepad 1 B : Turn to align to Hive");
            telemetry.addLine("Gamepad 1 X : Cycle Target Hive");
            telemetry.addLine("Gamepad 1 A : Auto Set Shooter RPM from Distance");
            telemetry.addLine("Left Stick Y / Right Stick X : Manual Drive");
            telemetry.addLine("Right Trigger : Shooter");

            telemetry.update();
        }

        shooter.stop();
    }
}
