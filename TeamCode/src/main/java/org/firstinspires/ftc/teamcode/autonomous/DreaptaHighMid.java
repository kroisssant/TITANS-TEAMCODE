package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Automatisms;
import org.firstinspires.ftc.teamcode.util.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;


@Autonomous (name = "dreaptaHTech")
@Disabled
public class DreaptaHighMid extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(1.5F, new int[]{0, 1, 2});
    protected OpenCvWebcam webcam;

    public int target = -1;
    protected boolean cameraOK = true;

    public SampleMecanumDrive drive;
    private Automatisms automatism;

    public Lifter lifter;
    public IntakeVechi intakeVechi;

    public Lifter.LIFTER_LEVEL currentLifterState;
    public IntakeVechi.INTAKE_STATE currentIntakeState;

    protected Thread intakeThread, lifterThread;

    static Pose2d initialPose = new Pose2d(36, -63, Math.toRadians(270));

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000); //3000
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                try {
                    Thread.sleep(1000); // always wait before pressing start
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("camera failed to open:", errorCode);
                telemetry.update();
                cameraOK = false;
            }
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intakeVechi = new IntakeVechi(hardwareMap, true);
        ElapsedTime timer = new ElapsedTime();

        initDetection();

        automatism = new Automatisms(lifter, intakeVechi);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = IntakeVechi.INTAKE_STATE.INIT;

        intakeVechi.spinFor(0, 50, 0.1); //bug

        // Creating trajectories
        TrajectorySequence toHighPreload = toHighPreload();

        TrajectorySequence toCone1 = toConeOne(toHighPreload);
        TrajectorySequence toMid1 = toMidOne(toCone1);

        TrajectorySequence toCone2 = toConeTwo(toMid1);
        TrajectorySequence toMid2 = toMidTwo(toCone2);

        TrajectorySequence toCone3 = toConeThree(toMid2);
        TrajectorySequence toMid3 = toMidThree(toCone3);

        TrajectorySequence toCone4 = toConeFour(toMid3);
        TrajectorySequence toMid4 = toMidFour(toCone4);

        TrajectorySequence toPark1 = toParkOne(toMid4);
        TrajectorySequence toPark2 = toParkTwo(toMid4);
        TrajectorySequence toPark3 = toParkThree(toMid4);

        if (cameraOK) {
            pipeline.startAprilTagDetection();
            telemetry.addLine("Created trajectories");
            telemetry.addLine("Webcam Ok");
            telemetry.addLine("Ready! Press Play");
        } else {
            telemetry.addLine("Webcam failed, please RESTART!");
        }

        telemetry.update();

        waitForStart();
        timer.reset();

        target = pipeline.targetFound;
        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        telemetry.addData("Sleeve", target + 1);
        telemetry.update();

        intakeThread = new Thread(intakeVechi);
        lifterThread = new Thread(lifter);

        lifterThread.start();
        intakeThread.start();

        drive.setPoseEstimate(initialPose);

        intakeVechi.setCRServosPow(0.1);


        lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.LOW.ticks);
        intakeVechi.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
        currentLifterState = Lifter.LIFTER_LEVEL.LOW;
        currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;

        intakeVechi.paConGoToPositionThread(0.8, 200);  // IF IT AIN'T BROKEN DON'T FIX IT

        drive.followTrajectorySequence(toHighPreload);
        sleep(50);
        intakeVechi.paConInside(800);

        drive.followTrajectorySequence(toCone1);
        sleep(650);
        intakeVechi.setCRServosPow(0.1);

        drive.followTrajectorySequence(toMid1);
        sleep(50);

//        drive.followTrajectorySequence(toCone2);
//        sleep(700);
//
//        drive.followTrajectorySequence(toMid2);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone3);
//        sleep(600);
//
//        drive.followTrajectorySequence(toMid3);
//        sleep(50);

        if(timer.seconds() > 10) {
            telemetry.addLine("enough time... proceeding to stack");
            telemetry.update();
            drive.followTrajectorySequence(toCone4);
            sleep(500);

            drive.followTrajectorySequence(toMid4);
            sleep(40);
        } else {
            telemetry.addLine("skipping to parking");
            telemetry.update();
        }

        if (target == 0) {
                drive.followTrajectorySequence(toPark1);
        } else if (target == 2) {
                drive.followTrajectorySequence(toPark3);
        } else {
                drive.followTrajectorySequence(toPark2);
        }


        sleep(200);

        lifter.killLifterThread();
        intakeVechi.killIntakeThread();

    }

    TrajectorySequence toHighPreload() {
        return drive.trajectorySequenceBuilder(initialPose)
                .setReversed(true)

                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    lifter.setTargetTicks(0, 500);
                })

                .addSpatialMarker(new Vector2d(-36, -52), () -> {
//                    intake.paConOutside(63);
                    intakeVechi.paConInterpolateThread(70, 0, 0.45, 200, 22);
                    intakeVechi.paConInside(500);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    lifter.setTargetTicks(1500, Lifter.LIFTER_LEVEL.HIGH.ticks);
                    intakeVechi.swingSetTargetTicks(400, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    intakeVechi.swingSetTargetTicks(500, IntakeVechi.INTAKE_STATE.INSIDE.ticks);


                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;

                    intakeVechi.spinFor(2500, 800, -0.6);
                })

                .setVelConstraint(new TranslationalVelocityConstraint(55))
                .setAccelConstraint(new ProfileAccelerationConstraint(55))

                .lineToLinearHeading(new Pose2d(36.5, -20, Math.toRadians(270)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(55))
                .setAccelConstraint(new ProfileAccelerationConstraint(45))

//                .lineToLinearHeading(new Pose2d(-31.5, -9, Math.toRadians(245)))

//                .setVelConstraint(new TranslationalVelocityConstraint(46))
//                .setAccelConstraint(new ProfileAccelerationConstraint(30))

                .lineToLinearHeading(new Pose2d(26.7, -1.8, Math.toRadians(312)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeOne(@NonNull TrajectorySequence toHighPreload) {
        return drive.trajectorySequenceBuilder(toHighPreload.end())
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(600, Lifter.LIFTER_LEVEL.LOW.ticks - 150);

                    lifter.setTargetTicks(1800, Lifter.LIFTER_LEVEL.LOW.ticks - 900);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    lifter.setTargetTicks(2500, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    intakeVechi.setCRServosPow(0);
                    intakeVechi.spinFor(0, 2700, 0.5);
                    telemetry.addLine("c'mon baby we know you can do it!");
                    telemetry.update();
                })
//                .setReversed(false)
//                .setVelConstraint(new TranslationalVelocityConstraint(45))
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//
//                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
//
//                .resetConstraints()
//
//                .splineToConstantHeading(new Vector2d(-57.5, -12), Math.toRadians(180))

                //vlad:
                .setReversed(false)

                //.splineToSplineHeading(new Pose2d(-45, -11.811, Math.toRadians(180)), Math.toRadians(180))

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(35))

                .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(0)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(40))

                .lineToConstantHeading(new Vector2d(57, -9))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toMidOne(@NonNull TrajectorySequence toCone1) {
        return drive.trajectorySequenceBuilder(toCone1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                    intakeVechi.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.HIGH.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                    intakeVechi.spinFor(1600, 800, -0.8);
                })
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-42, -13, Math.toRadians(180)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(30))
//
//                .splineToSplineHeading(new Pose2d(-24.8, -21.3, Math.toRadians(116)), Math.toRadians(322),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(32)
//                )

                .setReversed(true)

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(40))

                .lineToLinearHeading(new Pose2d(40, -9, Math.toRadians(0)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(27.2, -2.7, Math.toRadians(308)))

                .resetConstraints()
                .build();
    }

    TrajectorySequence toConeTwo(@NonNull TrajectorySequence toMid1) {
        return drive.trajectorySequenceBuilder(toMid1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1500, Lifter.LIFTER_LEVEL.LOW.ticks - 900);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    lifter.setTargetTicks(2400, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intakeVechi.setCRServosPow(0);
                    intakeVechi.spinFor(0, 2500, -0.5);
                    telemetry.addLine("c'mon baby we know you can do it!");
                    telemetry.update();
                })
//                .setReversed(false)
//                .setVelConstraint(new TranslationalVelocityConstraint(45))
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//
//                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
//
//                .resetConstraints()
//
//                .splineToConstantHeading(new Vector2d(-57.5, -12), Math.toRadians(180))

                //vlad:
                .setReversed(false)

                //.splineToSplineHeading(new Pose2d(-45, -11.811, Math.toRadians(180)), Math.toRadians(180))

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(40))

                .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(0)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

                .lineToConstantHeading(new Vector2d(57.5, -9))

                .resetConstraints()
                .build();
    }

    TrajectorySequence toMidTwo(@NonNull TrajectorySequence toCone2) {
        return drive.trajectorySequenceBuilder(toCone2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                    intakeVechi.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(650, Lifter.LIFTER_LEVEL.HIGH.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                    intakeVechi.spinFor(1700, 800, 0.8);
                })
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-42, -13, Math.toRadians(180)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(30))
//
//                .splineToSplineHeading(new Pose2d(-24.8, -21.3, Math.toRadians(116)), Math.toRadians(322),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(32)
//                )

                .setReversed(true)

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(45))

                .lineToLinearHeading(new Pose2d(40, -9, Math.toRadians(0)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(27.2, -3.1, Math.toRadians(308)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeThree(@NonNull TrajectorySequence toHigh2) {
        return drive.trajectorySequenceBuilder(toHigh2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1500, Lifter.LIFTER_LEVEL.LOW.ticks - 900);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    lifter.setTargetTicks(2450, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intakeVechi.setCRServosPow(0);
                    intakeVechi.spinFor(0, 2500, -0.5);
                    telemetry.addLine("c'mon baby we know you can do it!");
                    telemetry.update();
                })
//                .setReversed(false)
//                .setVelConstraint(new TranslationalVelocityConstraint(45))
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//
//                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
//
//                .resetConstraints()
//
//                .splineToConstantHeading(new Vector2d(-57.5, -12), Math.toRadians(180))

                //vlad:
                .setReversed(false)

                //.splineToSplineHeading(new Pose2d(-45, -11.811, Math.toRadians(180)), Math.toRadians(180))

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(40))

                .lineToLinearHeading(new Pose2d(45, -9, Math.toRadians(0)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

                .lineToConstantHeading(new Vector2d(57.6, -8.5))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toMidThree(@NonNull TrajectorySequence toCone3) {
        return drive.trajectorySequenceBuilder(toCone3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                    intakeVechi.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(650, Lifter.LIFTER_LEVEL.HIGH.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                    intakeVechi.spinFor(1700, 800, 0.8);
                })
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-42, -13, Math.toRadians(180)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(30))
//
//                .splineToSplineHeading(new Pose2d(-24.8, -21.3, Math.toRadians(116)), Math.toRadians(322),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(32)
//                )

                .setReversed(true)

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(45))

                .lineToLinearHeading(new Pose2d(40, -9, Math.toRadians(0)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(27.2, -2.9, Math.toRadians(309)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeFour(@NonNull TrajectorySequence toHigh3) {
        return drive.trajectorySequenceBuilder(toHigh3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1500, Lifter.LIFTER_LEVEL.LOW.ticks - 1200);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    lifter.setTargetTicks(2400, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intakeVechi.setCRServosPow(0);
                    intakeVechi.spinFor(0, 2500, -0.5);
                    telemetry.addLine("c'mon baby we know you can do it!");
                    telemetry.update();
                })
//                .setReversed(false)
//                .setVelConstraint(new TranslationalVelocityConstraint(45))
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//
//                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
//
//                .resetConstraints()
//
//                .splineToConstantHeading(new Vector2d(-57.5, -12), Math.toRadians(180))

                //vlad:
                .setReversed(false)

                //.splineToSplineHeading(new Pose2d(-45, -11.811, Math.toRadians(180)), Math.toRadians(180))

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(40))

                .lineToLinearHeading(new Pose2d(45, -10, Math.toRadians(0)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

                .lineToConstantHeading(new Vector2d(57.5, -8.5))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toMidFour(@NonNull TrajectorySequence toCone4) {
        return drive.trajectorySequenceBuilder(toCone4.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                    intakeVechi.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.HIGH.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                    intakeVechi.spinFor(1700, 800, 0.8);
                })
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-42, -13, Math.toRadians(180)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(30))
//
//                .splineToSplineHeading(new Pose2d(-24.8, -21.3, Math.toRadians(116)), Math.toRadians(322),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(32)
//                )

                .setReversed(true)

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(45))

                .lineToLinearHeading(new Pose2d(40, -9, Math.toRadians(0)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(27.2, -2.95, Math.toRadians(309)))

                .resetConstraints()

                .build();
    }


    TrajectorySequence toParkOne(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToLinearHeading(new Pose2d(55.5, -11, Math.toRadians(0)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(60)
                )

                .build();
    }

    TrajectorySequence toParkTwo(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.DOWN.ticks);
                    intakeVechi.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(36.5, -15, Math.toRadians(0)), Math.toRadians(270),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(60)
                )

                .build();
    }

    TrajectorySequence toParkThree(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(270)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(60)
                )

                .build();
    }
}
