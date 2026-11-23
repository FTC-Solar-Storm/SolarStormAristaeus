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

    private Pose2d startPose = new Pose2d(-59.7,16.26 , Math.toRadians(180));
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