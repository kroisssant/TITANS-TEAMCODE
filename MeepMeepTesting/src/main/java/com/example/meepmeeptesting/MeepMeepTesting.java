package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class MeepMeepTesting {

    static public int pozitieParcare = 1;  //1 2 sau 3

    // Red - Blue Terminal initial pose DREAPTA
    static Pose2d initialPoseRedBlueTerminal = new Pose2d(36, -63, Math.toRadians(270));

    // Red Red Terminal initial pose STANGA
    static Pose2d initialPoseRedRedTerminal = new Pose2d(-35, -63, Math.toRadians(270));


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, 3.8, 3.8, 13)
                .setDimensions(15.3543, 14.56).build();

        //myFirstBot.followTrajectorySequence(REDAllianceBlueTerminal(myFirstBot, initialPoseRedBlueTerminal, pozitieParcare ));
        //myFirstBot.followTrajectorySequence(REDAllianceRedTerminal(myFirstBot, initialPoseRedRedTerminal, pozitieParcare));

      //myFirstBot.followTrajectorySequence(stangaNormal(myFirstBot, initialPoseRedRedTerminal));
      myFirstBot.followTrajectorySequence(stangaRebuild(myFirstBot, initialPoseRedRedTerminal));
        //myFirstBot.followTrajectorySequence(stangaMid(myFirstBot, initialPoseRedRedTerminal));
        //myFirstBot.followTrajectorySequence(dreaptaMid(myFirstBot, initialPoseRedRedTerminal));
        //myFirstBot.followTrajectorySequence(dreaptaMidSecondoOption(myFirstBot, initialPoseRedBlueTerminal));


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .start();
    }

    public static TrajectorySequence stangaRebuild(RoadRunnerBotEntity bot, Pose2d initialPose){
        DriveShim drive = bot.getDrive();

        return drive.trajectorySequenceBuilder(initialPose)
                .lineToConstantHeading(new Vector2d(-35, -12))
                .turn(Math.toRadians(80))
                .lineToConstantHeading(new Vector2d(-55, -12)).build();
    }

    public static TrajectorySequence dreaptaMidSecondoOption(RoadRunnerBotEntity bot, Pose2d initialPose) {
        DriveShim drive = bot.getDrive();
        return drive.trajectorySequenceBuilder(initialPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(34, -13, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(27, -1, Math.toRadians(340)))


                //----------FIRST CYCLE-------
                //to cone
                .setReversed(false)

                .splineToSplineHeading(new Pose2d(50, -11.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(58.5, -11.5))

                //to mid
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                //.lineToSplineHeading(new Pose2d(24.8, -21.3, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(26, -20, Math.toRadians(90-35)), Math.toRadians(270 - 55))

//                //parking 1
//                .setReversed(false)
////                .splineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)), Math.toRadians(0))
////                .lineTo(new Vector2d(58, -12))
//
//
//                //parking 2
//                .setReversed(false)
//                //          .splineToSplineHeading(new Pose2d(35, -17, Math.toRadians(0)), Math.toRadians(270))
//
//                //parking 3
////                .setReversed(false)
//                //.splineToSplineHeading(new Pose2d(54, -13, Math.toRadians(0)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(12, -9, Math.toRadians(90)), Math.toRadians(180))



                .build();

    }


    public static TrajectorySequence dreaptaNormal(RoadRunnerBotEntity bot, Pose2d initialPose) {
        DriveShim drive = bot.getDrive();
        return drive.trajectorySequenceBuilder(initialPose).setReversed(true)

                .lineToLinearHeading(new Pose2d(-36.5, -30, Math.toRadians(270)))

                .setAccelConstraint(new ProfileAccelerationConstraint(55))
                .setVelConstraint(new TranslationalVelocityConstraint(45))

                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))

//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(0)))
//                .lineToConstantHeading(new Vector2d(58.26, -10))
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(40, -9, Math.toRadians(0)))

                .build();

    }

    public static TrajectorySequence dreaptaMid(RoadRunnerBotEntity bot, Pose2d initialPose) {
        DriveShim drive = bot.getDrive();
        return drive.trajectorySequenceBuilder(initialPose)


                .lineToLinearHeading(new Pose2d(-36.5, -20, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-31.5, -10, Math.toRadians(245)))
                .lineToLinearHeading(new Pose2d(-27, -2, Math.toRadians(232)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-58.2, -11))
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-11.5, -13.7, Math.toRadians(270)), Math.toRadians(0))

                .build();

    }

    public static TrajectorySequence stangaMid(RoadRunnerBotEntity bot, Pose2d initialPose) {
        DriveShim drive = bot.getDrive();
        return drive.trajectorySequenceBuilder(initialPose)

                .setReversed(false)

                .lineToLinearHeading(new Pose2d(-35, -32, Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-27, -1, Math.toRadians(208)), Math.toRadians(61))                //----------FIRST CICLE-------

                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(180)))

                //to mid
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-27.2, -20, Math.toRadians(120)), Math.toRadians(320))


                //----------SECOND CICLE-------
                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                //to mid
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-27.2, -20, Math.toRadians(120)), Math.toRadians(320))

                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                //to mid
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-27.2, -20, Math.toRadians(120)), Math.toRadians(320))

                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                //to mid
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-27.2, -20, Math.toRadians(120)), Math.toRadians(320))

                //etc

                //parking 1
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
//                .lineTo(new Vector2d(-58, -12))

                //parking 2
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(-35, -14, Math.toRadians(90)), Math.toRadians(180))

                //parking 3
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-12, -11, Math.toRadians(90)), Math.toRadians(0))

                .build();

    }

    public static TrajectorySequence stangaNormal(RoadRunnerBotEntity bot, Pose2d initialPose) {
        DriveShim drive = bot.getDrive();
        return drive.trajectorySequenceBuilder(initialPose)
//                .setReversed(false)
//                .lineToSplineHeading(new Pose2d(-33, -33, Math.toRadians(283)))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(-27.5, -1.5, Math.toRadians(217)), Math.toRadians(62))
                .setReversed(false)
                //.lineToLinearHeading(new Pose2d(-34, -28, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-35, -32, Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-27, -1, Math.toRadians(208)), Math.toRadians(61))                //----------FIRST CICLE-------

                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(180)))

                //to high
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-28.5, -4, Math.toRadians(220)), Math.toRadians(62))


                //----------SECOND CICLE-------
                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                //to high
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-28.5, -4, Math.toRadians(220)), Math.toRadians(62))

                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                //to high
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-28.5, -4, Math.toRadians(220)), Math.toRadians(62))

                //to cone
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -12))

                //to high
                .setReversed(true)
                .lineTo(new Vector2d(-50, -12))
                .splineToSplineHeading(new Pose2d(-28.5, -4, Math.toRadians(220)), Math.toRadians(62))

                //etc

                //parking 1
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-58, -12))

                //parking 2
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(-35, -17, Math.toRadians(270)), Math.toRadians(270))

                //parking 3
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(-12, -15, Math.toRadians(270)), Math.toRadians(0))

                .build();

    }

    public static TrajectorySequence REDAllianceBlueTerminal(RoadRunnerBotEntity bot, Pose2d initialPose, int pozitieParcare) {
        DriveShim drive = bot.getDrive();

        if (pozitieParcare == 1) {
            return drive.trajectorySequenceBuilder(initialPose)

                    //-----GO HIGH AND LEAVE PRELOAD--------
                    //.lineTo(new Vector2d(34, -40))
                    //.splineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)), Math.toRadians(90)) //-23y, 90 2math.torad

                    .lineToSplineHeading(new Pose2d(34.5, -6, Math.toRadians(145)))

                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //------PARKING FROM HIGH to pos 1--------
                    .setReversed(true)
                    //.splineToLinearHeading(new Pose2d(13, -15, Math.toRadians(0)), Math.toRadians(-150))

                    //sa vedem care metda de parcare e cea mai eficienta in combinatie cu intake ul ala
                    .splineToLinearHeading(new Pose2d(12, -15, Math.toRadians(85)), Math.toRadians(0))

                    .build();

        } else if (pozitieParcare == 2) {

            return drive.trajectorySequenceBuilder(initialPose)

                    //-----GO MID AND LEAVE PRELOAD--------

                    //.lineTo(new Vector2d(34, -40))
                    //.splineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)), Math.toRadians(90)) //-23y, 90 2math.torad

                    //de fapt, aici o sa incerc sa pun si preload-ul pe high
                    //sper sa nu omoare odometria daca atinge ground-ul, e de testat
                    .lineToSplineHeading(new Pose2d(34, -7, Math.toRadians(145)))

                    //.lineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //------PARKING FROM HIGH to pos 2--------

                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(36, -19, Math.toRadians(90)))
                    .build();
        }

//default pos 3
        return drive.trajectorySequenceBuilder(initialPose)

                //-----GO MID AND LEAVE PRELOAD--------
                //.lineTo(new Vector2d(34, -40))
                //.splineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)), Math.toRadians(90)) //-23y, 90 2math.torad

                //----GO HIGH AND LEAVE PRELOAD---- dar e de testat
                .lineToSplineHeading(new Pose2d(34, -7, Math.toRadians(145)))

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(2500);
                    // intake.goBack(wait, pos) sa se duca in spate la o pozitie
                    //intake.setReverseSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 1500);
                })

                //---------GO COLLECT CONES (1) ------
                .setReversed(true)
                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK -----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //intake.goBack(wait, pos)
                    //intake.setReversedSpin(wait, 800);
                })

                //---------GO COLLECT CONES (2) ------
                .setReversed(true)
                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK -----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //intake.goBack(wait, pos)
                    //intake.setReversedSpin(wait, 800);
                })

                //------PARKING FROM HIGH to pos 3 default--------
                //.lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(58, -13), Math.toRadians(0))

                .build();
    }

    public static TrajectorySequence REDAllianceRedTerminal(RoadRunnerBotEntity bot, Pose2d initialPoseRedRed, int pozitieParcare){
        DriveShim drive = bot.getDrive();

        if(pozitieParcare == 1){
            return drive.trajectorySequenceBuilder(initialPoseRedRedTerminal)

                    //-----GO MID AND LEAVE PRELOAD--------
                    //.lineTo(new Vector2d(-34, -40))
                    //.splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(-30)), Math.toRadians(90)) //-23y, 90 2math.torad

                    //------GO HIGH AND LEAVE PRELOAD-----
                    .lineToSplineHeading(new Pose2d(-34, -7, Math.toRadians(45)))

                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    //.splineTo(new Vector2d(-32, -18), Math.toRadians(-30)) //40, 50
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
                    //.UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

//
//                    //------PARKING FROM HIGH to pos 1--------
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(0)), Math.toRadians(-170))


                    .build();

        } else if(pozitieParcare == 2){
            return drive.trajectorySequenceBuilder(initialPoseRedRedTerminal)

                    //-----GO MID AND LEAVE PRELOAD--------
                    //.lineTo(new Vector2d(-34, -40))
                    //.splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(-30)), Math.toRadians(90)) //-23y, 90

                    //------GO HIGH AND LEAVE PRELOAD-----
                    .lineToSplineHeading(new Pose2d(-34, -7, Math.toRadians(45)))

                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(45)), Math.toRadians(50))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

//
//                    //------PARKING FROM HIGH to pos 2--------
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(0)), Math.toRadians(-170))


                    .build();

        }
        return drive.trajectorySequenceBuilder(initialPoseRedRedTerminal)

                //-----GO MID AND LEAVE PRELOAD--------
                //.lineTo(new Vector2d(-34, -40))
                //.splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(-30)), Math.toRadians(90)) //-23y, 90 2math.torad

                //------GO HIGH AND LEAVE PRELOAD-----
                .lineToSplineHeading(new Pose2d(-34, -7, Math.toRadians(45)))

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(2500);
                    //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                    //intake.setReverseSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 1500);
                })

                //---------GO COLLECT CONES (1) ------
                .setReversed(true)
                .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK -----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

                //---------GO COLLECT CONES (2) ------
                .setReversed(true)
                .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK + parking 3-----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })


                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, -16, Math.toRadians(90)), Math.toRadians(-40))
                .build();}}
    