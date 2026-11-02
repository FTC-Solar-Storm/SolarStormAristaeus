package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter Test")
public class shootertest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        Shooter shooter = new Shooter(hardwareMap, telemetry, drive);

        int selectedPIDF = 0;

        boolean lastLeft = false;
        boolean lastRight = false;
        boolean lastA = false;
        boolean lastB = false;

        waitForStart();

        while (opModeIsActive()) {

            // FLYWHEEL: GAMEPAD 1
            if (gamepad1.right_trigger > 0.1) {
                shooter.update(gamepad1.dpad_up, gamepad1.dpad_down);
            } else {
                shooter.stop();
            }

            // RIGHT HOOD: GAMEPAD 1
            shooter.adjustRightHood(gamepad1.y, gamepad1.x);

            // LEFT HOOD: GAMEPAD 2
            shooter.adjustLeftHood(gamepad2.y, gamepad2.x);

            // AUTO-AIM: GAMEPAD 1 B
            shooter.updateAutoAim(gamepad1.b);

            // SELECT PIDF: GAMEPAD 2 D-PAD
            if (gamepad2.dpad_right && !lastRight) {
                selectedPIDF = (selectedPIDF + 1) % 4;
            }

            if (gamepad2.dpad_left && !lastLeft) {
                selectedPIDF = (selectedPIDF + 3) % 4;
            }

            lastRight = gamepad2.dpad_right;
            lastLeft = gamepad2.dpad_left;

            // CHANGE PIDF: GAMEPAD 2 A/B
            double change = 0;

            if (gamepad2.a && !lastA) change = -0.1;
            if (gamepad2.b && !lastB) change = 0.1;

            lastA = gamepad2.a;
            lastB = gamepad2.b;

            if (change != 0) {
                switch (selectedPIDF) {
                    case 0:
                        Shooter.FLYWHEEL_P = Math.max(0, Shooter.FLYWHEEL_P + change);
                        break;
                    case 1:
                        Shooter.FLYWHEEL_I = Math.max(0, Shooter.FLYWHEEL_I + change);
                        break;
                    case 2:
                        Shooter.FLYWHEEL_D = Math.max(0, Shooter.FLYWHEEL_D + change);
                        break;
                    case 3:
                        Shooter.FLYWHEEL_F = Math.max(0, Shooter.FLYWHEEL_F + change);
                        break;
                }

                shooter.applyPIDF();
            }

            // TELEMETRY
            telemetry.addLine("========== SHOOTER ==========");
            telemetry.addData("Enabled", gamepad1.right_trigger > 0.1);
            telemetry.addData("Target Velocity", shooter.getTargetVelocity());
            telemetry.addData("Flywheel 1 Velocity", shooter.getFlywheel1Velocity());
            telemetry.addData("Flywheel 2 Velocity", shooter.getFlywheel2Velocity());
            telemetry.addData("Average Velocity", shooter.getCurrentVelocity());

            telemetry.addLine("========== PIDF ==========");
            telemetry.addData("Selected", new String[]{"P", "I", "D", "F"}[selectedPIDF]);
            telemetry.addData("P", Shooter.FLYWHEEL_P);
            telemetry.addData("I", Shooter.FLYWHEEL_I);
            telemetry.addData("D", Shooter.FLYWHEEL_D);
            telemetry.addData("F", Shooter.FLYWHEEL_F);

            telemetry.addLine("========== HOODS ==========");
            telemetry.addData("Right Hood Position", shooter.getRightHoodPosition());
            telemetry.addData("Left Hood Position", shooter.getLeftHoodPosition());

            telemetry.addLine("========== AUTO AIM ==========");
            telemetry.addData("Enabled", shooter.isAutoAimEnabled());
            telemetry.addData("Tag ID", shooter.getSelectedTagId());
            telemetry.addData("Tag Distance", shooter.getSelectedTagDistance());
            telemetry.addData("Tag Angle", shooter.getSelectedTagAngle());

            telemetry.addLine("========== CONTROLS ==========");
            telemetry.addData("Flywheel", "Gamepad 1: Hold RT");
            telemetry.addData("Target Speed", "Gamepad 1: D-pad Up/Down");
            telemetry.addData("Right Hood", "Gamepad 1: Y/X");
            telemetry.addData("Left Hood", "Gamepad 2: Y/X");
            telemetry.addData("Auto Aim", "Gamepad 1: B");
            telemetry.addData("Select PIDF", "Gamepad 2: D-pad Left/Right");
            telemetry.addData("Adjust PIDF", "Gamepad 2: B increases, A decreases");

            telemetry.update();
        }

        shooter.shutdown();
    }
}