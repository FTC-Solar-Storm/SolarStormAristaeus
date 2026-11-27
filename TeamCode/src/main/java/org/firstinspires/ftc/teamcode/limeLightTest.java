package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LimeCam;

@TeleOp(name = "LimeTester")
public class limeLightTest extends LinearOpMode {
    //private Limelight3A limelight;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimeCam cam3A = new LimeCam(hardwareMap);
        waitForStart();

        // Reset starting position to (0, 0, 0)

        while (opModeIsActive()) {



            // ==========================================
            // TELEMETRY
            // ==========================================

            if (cam3A.seeBlueFront()) {
                telemetry.addLine("I can see Blue Hive near Audience");
            }
            if (cam3A.seeBlueBack()) {
                telemetry.addLine("I can see Blue Hive opposite of Audience");
            }

            if (cam3A.seeRedFront()) {
                telemetry.addLine("I can see Red Hive near Audience");
            }
            if (cam3A.seeRedBack()) {
                telemetry.addLine("I can see Red Hive opposite of Audience");
            }

            boolean [] tagList= cam3A.tagList;
            telemetry.addLine("I can see following tags: ");
            for (int i=0; i<16; i++){
                if (tagList[i]) {
                    telemetry.addData("Tag # ", i+30);
                }
            }




            telemetry.update();
        }


    }
}
