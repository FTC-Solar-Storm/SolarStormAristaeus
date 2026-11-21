package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter PID Tuning")
public class PIDControl extends LinearOpMode {

    // INITIAL PIDF VALUES`
    private double P = 10.0;
    private double I = 3.0;
    private double D = 0.0;
    private double F = 12.0;

    private double targetVelocity = 1700;

    // ADJUSTMENT STEPS
    private double pStep = 0.5;
    private double iStep = 0.1;
    private double dStep = 0.1;
    private double fStep = 0.5;

    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;

    // BUTTON STATES
    private boolean previousUp, previousDown;
    private boolean previousRight, previousLeft;
    private boolean previousX, previousB;
    private boolean previousY, previousA;

    @Override
    public void runOpMode() {

        // SEND TELEMETRY TO DRIVER STATION AND DASHBOARD
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // INITIALIZE MOTORS
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");

        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();

        telemetry.addLine("Shooter PID Tuning Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // P CONTROL
            // =========================

            if (gamepad1.dpad_up && !previousUp) {
                P += pStep;
            }

            if (gamepad1.dpad_down && !previousDown) {
                P = Math.max(0, P - pStep);
            }

            // =========================
            // I CONTROL
            // =========================

            if (gamepad1.dpad_right && !previousRight) {
                I += iStep;
            }

            if (gamepad1.dpad_left && !previousLeft) {
                I = Math.max(0, I - iStep);
            }

            // =========================
            // D CONTROL
            // =========================

            if (gamepad1.x && !previousX) {
                D += dStep;
            }

            if (gamepad1.b && !previousB) {
                D = Math.max(0, D - dStep);
            }

            // =========================
            // F CONTROL
            // =========================

            if (gamepad1.y && !previousY) {
                F += fStep;
            }

            if (gamepad1.a && !previousA) {
                F = Math.max(0, F - fStep);
            }

            // UPDATE BUTTON STATES
            previousUp = gamepad1.dpad_up;
            previousDown = gamepad1.dpad_down;
            previousRight = gamepad1.dpad_right;
            previousLeft = gamepad1.dpad_left;

            previousX = gamepad1.x;
            previousB = gamepad1.b;
            previousY = gamepad1.y;
            previousA = gamepad1.a;

            // APPLY PIDF
            applyPIDF();

            // =========================
            // SHOOTER CONTROL
            // =========================

            if (gamepad1.right_bumper) {
                rightShooter.setVelocity(targetVelocity);
                leftShooter.setVelocity(targetVelocity);
            }

            if (gamepad1.left_bumper) {
                rightShooter.setVelocity(0);
                leftShooter.setVelocity(0);
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addLine("===== SHOOTER PID TUNING =====");

            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("F", F);

            telemetry.addLine("-----------------------------");

            telemetry.addData("Target Velocity", targetVelocity);

            telemetry.addData("Right Velocity",
                    rightShooter.getVelocity());

            telemetry.addData("Left Velocity",
                    leftShooter.getVelocity());

            telemetry.addData("Right Error",
                    targetVelocity - rightShooter.getVelocity());

            telemetry.addData("Left Error",
                    targetVelocity - leftShooter.getVelocity());

            telemetry.addLine("-----------------------------");

            telemetry.addLine("DPAD UP/DOWN: P");
            telemetry.addLine("DPAD LEFT/RIGHT: I");
            telemetry.addLine("X/B: D");
            telemetry.addLine("Y/A: F");
            telemetry.addLine("RB: Spin");
            telemetry.addLine("LB: Stop");

            telemetry.update();
        }

        rightShooter.setVelocity(0);
        leftShooter.setVelocity(0);
    }

    // =========================
    // APPLY PIDF
    // =========================

    private void applyPIDF() {

        PIDFCoefficients pidf = new PIDFCoefficients(
                P,
                I,
                D,
                F
        );

        rightShooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        leftShooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );
    }
}