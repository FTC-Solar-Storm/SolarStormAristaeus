package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * Intake subsystem - controls intake motor and gate servo
 */
public final class Intake {

    public final Telemetry telemetry;
    public DcMotorEx intakeMotor;
    public DcMotor transfer;


    public CRServo right, left;
    public Servo gate;

    // ==================== CONSTRUCTOR ====================
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;


        right = hardwareMap.get(CRServo.class, "rightIntakeServo");
        left = hardwareMap.get(CRServo.class, "leftIntakeServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");


    }

    public void moveGate(boolean gatebutton){
        if (gatebutton) {

            if (gate.getPosition() == 0.35) {
                gate.setPosition(0);
            } else {
                gate.setPosition(0.35);
            }
        }
    }

}