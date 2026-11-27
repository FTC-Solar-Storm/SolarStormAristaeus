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

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.TankDrive;


@Autonomous(name = "DominantHiveAuto", group = "Autonomous")
public class dominantHiveAuto extends LinearOpMode {


    private TankDrive drive;
    private AutoInfoStorage info;

    private Pose2d startPose = new Pose2d(-59.7,16.26 , Math.toRadians(180));
    private Pose2d backup = new Pose2d(-53, 16, Math.toRadians(180));

    private Pose2d backupTurn = new Pose2d(-53, 16, Math.toRadians(103));


    private Pose2d cornerReady = new Pose2d(-59,39.9,Math.toRadians(103));

    private Pose2d cornerEnd = new Pose2d(-59.7, 61.6, Math.toRadians(94));

    private Pose2d backupFStart = new Pose2d(-21, 38.9, Math.toRadians(-139));

    private Pose2d flowerReady = new Pose2d(-21, 38.9, Math.toRadians(90));

    private Pose2d flowerIntake = new Pose2d(, , Math.toRadians());

    private Pose2d startPose = new Pose2d(, , Math.toRadians());

    private Pose2d startPose = new Pose2d(, , Math.toRadians());
    private Pose2d startPose = new Pose2d(, , Math.toRadians());

    private Pose2d startPose = new Pose2d(, , Math.toRadians());



    @Override
    public void runOpMode() {









    }

}