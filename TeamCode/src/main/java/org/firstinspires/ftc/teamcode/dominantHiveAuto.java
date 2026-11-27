package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.CameraLL;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.TankDrive;


@Autonomous(name = "DominantHiveAuto", group = "Autonomous")
public class dominantHiveAuto extends LinearOpMode {


    private TankDrive drive;
    private AutoInfoStorage info;
    private SSMyRobot myRobot;
    private Shooter shooter;
    private Intake intake;
    private CameraLL camera;
    private Pose2d startPose = new Pose2d(-59.7,16.26 , Math.toRadians(180));
    private Pose2d backup = new Pose2d(-53, 16, Math.toRadians(180));
    //public ElapsedTime generalTimer = new ElapsedTime();

    private Pose2d backupTurn = new Pose2d(-53, 16, Math.toRadians(103));


    private Pose2d cornerReady = new Pose2d(-59,39.9,Math.toRadians(103));

    private Pose2d cornerEnd = new Pose2d(-59.7, 61.6, Math.toRadians(94));

    private Pose2d backupFStart = new Pose2d(-21, 38.9, Math.toRadians(-139));

    private Pose2d flowerReady = new Pose2d(-21, 38.9, Math.toRadians(90));





    @Override
    public void runOpMode() {


        drive = new TankDrive(hardwareMap, startPose);

        info = new AutoInfoStorage();
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry, drive);
        camera = new CameraLL(hardwareMap);

        myRobot = new SSMyRobot(hardwareMap, drive, intake, shooter, startPose, camera);

        intake.nectarGate.setPosition(1.0);
        intake.gate.setPosition(0.4);
        camera.Alliance = "Blue";
        shooter.setHiveTarget(Shooter.HiveTarget.BLUE_HIVE_BOTTOM);
        waitForStart();

        info.autoEndPose = startPose;
        myRobot.updateRobot();

        Actions.runBlocking(
                new ParallelAction(
                        myRobot.updateRobot(),
                        new SequentialAction(

                                myRobot.autoSetShooterFromDistance(),
                                myRobot.spinFlywheels(),
                                myRobot.openGate(),
                                myRobot.setIntakePower(0.6)

                )


        ));










        while (opModeIsActive()) {
         telemetry.update();
        }

    }

}