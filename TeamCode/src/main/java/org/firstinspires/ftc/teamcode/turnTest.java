package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "turnTest")
public class turnTest extends LinearOpMode {

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
        double turnAngle=0;
        double forwardMove=0;
        while (opModeIsActive()) {

            // Central Subsystem Update (Pose, Distance, Target Velocity Steps)
            shooter.update(gamepad1.dpad_up, gamepad1.dpad_down);

            // Drive Inputs
            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;


            // Hive Auto-Align (Button B = Auto-set shooter & align with 5s timeout / toggle off)
            boolean hiveAlignActive = shooter.updateHiveAutoAlign(gamepad2.bWasPressed() || gamepad1.bWasPressed());

            // Target Hive Selection & Auto Shooter Distance Config
            if (gamepad2.xWasPressed()) shooter.cycleHiveTarget();
            if (gamepad2.aWasPressed()) shooter.autoSetShooterFromDistance();
            if (gamepad1.dpadRightWasPressed()) turnAngle+=10;
            if (gamepad1.dpadLeftWasPressed()) turnAngle-=10;
            if (gamepad1.dpadUpWasPressed()) forwardMove+=1;
            if (gamepad1.dpadDownWasPressed()) forwardMove-=1;
            // Color Vision Auto Pickup
            boolean autoPickupActive = vision.updateAutoPickup(gamepad1.rightBumperWasPressed(), drive, intake);

            Pose2d currentPose = drive.localizer.getPose();
            if (gamepad1.xWasPressed()){
                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                                .turn(Math.toRadians(-turnAngle-Math.copySign(5,turnAngle)))
                                .build()
                );
            } else if (gamepad1.yWasPressed()){
                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                        .splineTo(new Vector2d(currentPose.position.x+forwardMove*Math.cos(currentPose.heading.toDouble()+Math.toRadians(turnAngle+Math.copySign(5,turnAngle)))
                                ,currentPose.position.y+forwardMove*Math.sin(currentPose.heading.toDouble()+Math.toRadians(turnAngle+Math.copySign(5,turnAngle))))
                                ,currentPose.heading.toDouble()+Math.toRadians(turnAngle+Math.copySign(5,turnAngle)))
                                .build()
                );
            }
            else {
                if (!autoPickupActive && !hiveAlignActive) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), turn));
                }
            }
            // Manual Drive


            // Gate Servo Toggle
            intake.moveGate(gamepad2.rightBumperWasPressed());

            // Manual Intake Motor
            double intakePower = gamepad1.left_trigger * 0.6;
            if (!vision.isAutoPickupActive()) {
                intake.intakeMotor.setPower(intakePower);
                intake.left.setPower(-intakePower);
                intake.right.setPower(intakePower);
            }

            // Shooter Flywheel Motors
            if (gamepad1.right_trigger > 0.1) {
                shooter.spin();
            } else {
                shooter.stop();
            }

            // Manual Hood Adjustments
            //shooter.adjustRightHood(gamepad1.yWasPressed(), gamepad1.aWasPressed());
            //shooter.adjustLeftHood(gamepad1.xWasPressed(), gamepad1.bWasPressed());

            // Apply PIDF
            shooter.applyPIDF();

            // ==========================================
            // TELEMETRY
            // ==========================================



            telemetry.addLine("========== ROBOT ODOM POSE ==========");
            telemetry.addData("Pose X (in)", "%.2f", currentPose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", currentPose.position.y);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Turn Angle is ", turnAngle);
            telemetry.addData("Forward Move value:", forwardMove);
            telemetry.addLine("\n========== CONTROLS ==========");
            telemetry.addLine("Left Stick Y / Right Stick X: Manual Drive");
            telemetry.addLine("Left Trigger: Manual Intake");
            telemetry.addLine("Right Bumper (Gamepad 1): Color Vision Auto Pickup");
            telemetry.addLine("Right Trigger: Spin Flywheels");
            telemetry.addLine("D-Pad Up/Down: Manual Speed Step");
            telemetry.addLine("Gamepad 1 Y/A: Right Hood Adjust");
            telemetry.addLine("Gamepad 1 X/B: Left Hood Adjust");
            telemetry.addLine("Gamepad 2 B / Gamepad 1 B: Auto Shooter & Hive Align (5s)");
            telemetry.addLine("Gamepad 2 X: Cycle Target Hive");
            telemetry.addLine("Gamepad 2 A: Auto-Set Shooter from Distance");
            telemetry.addLine("Gamepad 2 Right Bumper: Toggle Gate");

            telemetry.addLine("\n========== SHOOTER ==========");
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
