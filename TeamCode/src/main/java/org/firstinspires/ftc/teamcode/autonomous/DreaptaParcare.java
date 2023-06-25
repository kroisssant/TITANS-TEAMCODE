package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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

@Disabled
@Autonomous
public class DreaptaParcare extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(1.5F, new int[]{0, 1, 2});
    protected OpenCvWebcam webcam;

    public int target = -1;
    protected boolean cameraOK = true;

    public SampleMecanumDrive drive;
    private Automatisms automatism;

    public Lifter lifter;
    public IntakeVechi intake;

    public Lifter.LIFTER_LEVEL currentLifterState;
    public IntakeVechi.INTAKE_STATE currentIntakeState;

    protected Thread intakeThread, lifterThread;

    static Pose2d initialPoseLeft = new Pose2d(36, -63, Math.toRadians(270));

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
        intake = new IntakeVechi(hardwareMap, true);

        initDetection();
        timer.reset();

        automatism = new Automatisms(lifter, intake);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = IntakeVechi.INTAKE_STATE.INIT;

        intake.spinFor(0, 50, 0.1); //bug

        // Creating trajectories
        TrajectorySequence toHighPreload = toHighPreload();

        TrajectorySequence toCone1 = toConeOne(toHighPreload);
        TrajectorySequence toMid1 = toMidOne(toCone1);

        TrajectorySequence toPark1 = toParkOne(toMid1);
        TrajectorySequence toPark2 = toParkTwo(toMid1);
        TrajectorySequence toPark3 = toParkThree(toMid1);

        pipeline.startAprilTagDetection();

        if (cameraOK) {
            telemetry.addLine("Created trajectories");
            telemetry.addLine("Webcam Ok");
            telemetry.addLine("Ready! Press Play");
        } else {
            telemetry.addLine("Webcam failed, please RESTART!");
            telemetry.update();
            sleep(1000);
        }

        telemetry.update();

        waitForStart();

        target = pipeline.targetFound;

        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        telemetry.addData("Sleeve", target + 1);
        telemetry.update();

        intakeThread = new Thread(intake);
        lifterThread = new Thread(lifter);

        lifterThread.start();
        intakeThread.start();

        drive.setPoseEstimate(initialPoseLeft);

        intake.setCRServosPow(-0.1);


        lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.LOW.ticks);
        intake.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
        currentLifterState = Lifter.LIFTER_LEVEL.LOW;
        currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;

        intake.paConGoToPositionThread(0.8, 200);  // IF IT AIN'T BROKE DON'T FIX IT

        drive.followTrajectorySequence(toHighPreload);
        sleep(50);
        intake.paConInside(800);
        drive.followTrajectorySequence(toCone1);
        sleep(700);
        drive.followTrajectorySequence(toMid1);
        sleep(50);

        if(target == 0){
            drive.followTrajectorySequence(toPark1);
        } else if( target == 2){
            drive.followTrajectorySequence(toPark3);
        } else {
            drive.followTrajectorySequence(toPark2);
        }

        sleep(200);

        lifter.kill = true;
        intake.kill = true;

    }

    TrajectorySequence toHighPreload() {
        return drive.trajectorySequenceBuilder(initialPoseLeft)

                .UNSTABLE_addTemporalMarkerOffset(0.44, () -> {
                    lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.HIGH.ticks);
                    intake.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    intake.paConInterpolateThread(600, 0.7, 0.45, 700, 30); // IF IT AIN'T BROKE DON'T FIX IT

                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;

                    intake.spinFor(2300, 800, 0.6);
                })
                .setReversed(false)

               // .lineToLinearHeading(new Pose2d(37.5, -23, Math.toRadians(270)),
                .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(270)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(75, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(42))

                .splineToSplineHeading(new Pose2d(27, 0, Math.toRadians(320)), Math.toRadians(100),

               // .splineToSplineHeading(new Pose2d(27, 0.5, Math.toRadians(326)), Math.toRadians(120),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(25)
                )

                .build();

//                .UNSTABLE_addTemporalMarkerOffset(0.44, () -> {
//                    lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.HIGH.ticks);
//                    intake.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
//                    intake.paConInterpolateThread(600, 0.7, 0.45, 1000, 30); // IF IT AIN'T BROKE DON'T FIX IT
//
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
//
//                    intake.spinFor(2300, 800, 0.6);
//                })
//                .setReversed(false)
//
//                .lineToLinearHeading(new Pose2d(35, -25, Math.toRadians(270)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(75, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(42))
//
//                .splineToSplineHeading(new Pose2d(29.9, -1, Math.toRadians(318)), Math.toRadians(120),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(25)
//                )
//
//                .build();
    }

    TrajectorySequence toConeOne(@NonNull TrajectorySequence toHighPreload) {
        return drive.trajectorySequenceBuilder(toHighPreload.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1200, Lifter.LIFTER_LEVEL.LOW.ticks - 500);//1350

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;
                    intake.setCRServosPow(-0.5);
                    lifter.setTargetTicks(2600, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })
                .setReversed(false)
                .setVelConstraint(new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(new ProfileAccelerationConstraint(25))

                .splineToSplineHeading(new Pose2d(45, -10.5, Math.toRadians(0)), Math.toRadians(0))

                .resetVelConstraint()
                .resetAccelConstraint()

                .splineToConstantHeading(new Vector2d(60, -9.5), Math.toRadians(0))

                .build();
    }

    TrajectorySequence toMidOne(@NonNull TrajectorySequence toCone1) {
        return drive.trajectorySequenceBuilder(toCone1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                    intake.swingSetTargetTicks(200, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.MID.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                    intake.spinFor(1900, 800, 0.8);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(42, -13, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30))

                .splineToSplineHeading(new Pose2d(27, -20, Math.toRadians(90-25)), Math.toRadians(270 - 55),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(32)
                )

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
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(270)))
                .lineTo(new Vector2d(13, -12))

                .build();
    }

    TrajectorySequence toParkTwo(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    lifter.setTargetTicks(300, Lifter.LIFTER_LEVEL.DOWN.ticks);
                    intake.swingSetTargetTicks(400, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(38, -15, Math.toRadians(0)), Math.toRadians(270),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )

                .lineToLinearHeading(new Pose2d(38, -15, Math.toRadians(0)))

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
                .splineToSplineHeading(new Pose2d(60.5, -13, Math.toRadians(0)), Math.toRadians(0))


                .build();
    }
}
