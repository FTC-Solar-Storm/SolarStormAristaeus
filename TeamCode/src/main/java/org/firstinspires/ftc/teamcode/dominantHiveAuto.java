package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
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

    private boolean firstTilt = false;
    private TankDrive drive;
    private AutoInfoStorage info;
    private SSMyRobot myRobot;
    private Shooter shooter;
    private Intake intake;
    private CameraLL camera;
    private Pose2d startPose = new Pose2d(-59.7,16.26 , Math.toRadians(180));
    private Pose2d backup = new Pose2d(-50, 13, Math.toRadians(180));//new Pose2d(-53, 16, Math.toRadians(180));

    private Pose2d backupTurn = new Pose2d(-53.7, 16.7, Math.toRadians(75)); //heading 103 for corner

    private double forwardMove = 2;

    //private Pose2d cornerReady = new Pose2d(-57,39.9,Math.toRadians(103));

    //private Pose2d cornerEnd = new Pose2d(-57.7, 61.6, Math.toRadians(94));

    private Pose2d backupFStart = new Pose2d(-21, 38.9, Math.toRadians(-139));

    private Pose2d flowerReady = new Pose2d(-24, 36.9, Math.toRadians(100));

    private Pose2d flowerIntake = new Pose2d(-24.5, 55, Math.toRadians(90));
    private Pose2d flowerOut = new Pose2d(-24.5, 49, Math.toRadians(90));


    private Pose2d aprilCheck = new Pose2d(-11, 47.7, Math.toRadians(51));

    private Pose2d checkPoint = new Pose2d(31, 30, Math.toRadians(-24));





    @Override
    public void runOpMode() {


        drive = new TankDrive(hardwareMap, startPose);

        info = new AutoInfoStorage();
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry, drive);
        camera = new CameraLL(hardwareMap);

        myRobot = new SSMyRobot(hardwareMap, drive, intake, shooter, startPose, camera);

        intake.nectarGate.setPosition(0.9);
        intake.gate.setPosition(0.4);
        camera.Alliance = "Blue";
        shooter.setHiveTarget(Shooter.HiveTarget.BLUE_HIVE_BOTTOM);
        waitForStart();

        info.autoEndPose = startPose;
        //myRobot.updateRobot();
        drive.updatePoseEstimate();
        telemetry.addData("X: ", drive.localizer.getPose().position.x);
        telemetry.addData("Y: ", drive.localizer.getPose().position.y);
        telemetry.addData("Heading: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.update();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //new ParallelAction(
                        //myRobot.updateRobot(),
                        new SequentialAction(

                                myRobot.autoSetShooterFromDistance(),
                                myRobot.spinFlywheels(),
                                myRobot.openGate(),
                                new SleepAction(2),
                                myRobot.setIntakePower(0.6),
                                new SleepAction(2),
                                myRobot.setIntakePower(0.8),
                                myRobot.closeGate(),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .setReversed(true)
                                        .splineTo(backup.position, -70)
                                        //.turn(-77-5)
                                        //.setReversed(false)
                                       // .splineTo(backupTurn.position, backupTurn.heading)



                                        //.turn(77+5)
                                        .build(),

                                myRobot.setIntakePower(0.5),
                                myRobot.stopFlywheels()

                        )

                //)
        );
        drive.updatePoseEstimate();
        AutoInfoStorage.autoEndPose = drive.localizer.getPose();
        telemetry.addData("X: ", drive.localizer.getPose().position.x);
        telemetry.addData("Y: ", drive.localizer.getPose().position.y);
        telemetry.addData("Heading: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                //.splineTo(new Vector2d(drive.localizer.getPose().position.x + forwardMove *Math.cos(drive.localizer.getPose().heading.toDouble() + Math.toRadians(-60))
                                //                ,drive.localizer.getPose().position.y + forwardMove *Math.sin(drive.localizer.getPose().heading.toDouble() + Math.toRadians(-60)))
                                //        ,drive.localizer.getPose().heading.toDouble() + Math.toRadians(-60))
                                .splineTo(flowerReady.position, flowerReady.heading, new TranslationalVelConstraint(20))
                                .build(),
                        myRobot.lowerRamp()
                )


        );

        drive.updatePoseEstimate();
        AutoInfoStorage.autoEndPose = drive.localizer.getPose();



        Actions.runBlocking(
                        new SequentialAction(


                                drive.actionBuilder(drive.localizer.getPose())
                                        .splineTo(flowerIntake.position, flowerIntake.heading, new TranslationalVelConstraint(15))
                                        //.setReversed(true)
                                        //.splineTo(flowerReady.position, flowerReady.heading)
                                        .build(),
                                new SleepAction(2)
                        )
        );

        drive.updatePoseEstimate();
        AutoInfoStorage.autoEndPose = drive.localizer.getPose();


        Actions.runBlocking(
               // new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .setReversed(true)
                                .splineTo(flowerReady.position, Math.toRadians(-90), new TranslationalVelConstraint(15))
                                //.setReversed(false)
                                //.splineTo(flowerIntake.position, flowerIntake.heading, new TranslationalVelConstraint(10))
                                .build()
//                )



        );

        drive.updatePoseEstimate();
        AutoInfoStorage.autoEndPose = drive.localizer.getPose();


        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())

                                .splineTo(aprilCheck.position, aprilCheck.heading, new TranslationalVelConstraint(15))

                                .build(),
                        new SleepAction(2)
                )



        );


        drive.updatePoseEstimate();
        AutoInfoStorage.autoEndPose = drive.localizer.getPose();
        camera.audienceSide = false;

        if (!camera.detectTip()) {
            Actions.runBlocking(

                    drive.actionBuilder(drive.localizer.getPose())
                            .setReversed(true)
                            .splineTo(flowerReady.position, Math.toRadians(-140), new TranslationalVelConstraint(10))

                            .build()


            );
        } else  {
            Actions.runBlocking(

                    drive.actionBuilder(drive.localizer.getPose())

                            .splineTo(checkPoint.position, checkPoint.heading, new TranslationalVelConstraint(10))

                            .build()


            );
        }


        drive.updatePoseEstimate();
        AutoInfoStorage.autoEndPose = drive.localizer.getPose();

        while (opModeIsActive()) {
            telemetry.addData("X: ", drive.localizer.getPose().position.x);
            telemetry.addData("Y: ", drive.localizer.getPose().position.y);
            telemetry.addData("Heading: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
         telemetry.update();
        }

    }

}