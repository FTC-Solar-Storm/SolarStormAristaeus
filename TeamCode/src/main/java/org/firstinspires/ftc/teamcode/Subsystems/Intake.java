package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem - controls intake motor and gate servo
 */
public final class Intake {

    public final Telemetry telemetry;
    public DcMotorEx intakeMotor;
    public DcMotor transfer;



    public Servo flower;

    public Servo nectarGate;

    public CRServo right, left;
    public Servo gate;

    // ==================== CONSTRUCTOR ====================
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        right = hardwareMap.get(CRServo.class, "rightIntakeServo");
        left = hardwareMap.get(CRServo.class, "leftIntakeServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        flower = hardwareMap.get(Servo.class, "flower");
        nectarGate = hardwareMap.get(Servo.class, "nectarGate");
    }

    public boolean gateUp = true;

    public void moveGate(boolean gatebutton) {
        if (gatebutton) {
            if (gateUp) {
                gate.setPosition(0.4);
                gateUp = !gateUp;
            } else {
                gate.setPosition(0.2);
                gateUp = !gateUp;
            }
        }
    }

    public boolean flowerUp = true;

    public void moveFlower(boolean flowerbutton) {
        if (flowerbutton) {
            if (flowerUp) {
                flower.setPosition(0.3);
                flowerUp = !flowerUp;
            } else {
                flower.setPosition(0.85);
                flowerUp = !flowerUp;
            }
        }
    }

    public boolean nectarUp = true;

    public void moveNectar(boolean nectarbutton) {
        if (nectarbutton) {
            if (nectarUp) {
                nectarGate.setPosition(1);
                nectarUp = !nectarUp;
            } else {
                nectarGate.setPosition(0.83);
                nectarUp = !nectarUp;
            }
        }
    }


}
