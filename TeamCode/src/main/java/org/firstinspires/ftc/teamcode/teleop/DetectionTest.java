
package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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



@Autonomous (name = "testIntake")
public class DetectionTest extends LinearOpMode {
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

    static Pose2d initialPoseLeft = new Pose2d(-36, -63, Math.toRadians(270));

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initDetection();
        timer.reset();

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

        drive.setPoseEstimate(initialPoseLeft);

//        intake.setCRServosPow(-0.1);


//        lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.LOW.ticks);
//        intake.swingSetTargetTicks(200, Intake.INTAKE_STATE.INSIDE.ticks);
//        currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//        currentIntakeState = Intake.INTAKE_STATE.INSIDE;

//        intake.paConGoToPositionThread(0.8, 200);  // IF IT AIN'T BROKE DON'T FIX IT

        drive.followTrajectorySequence(toHighPreload);
        sleep(50);
//        intake.paConInside(800);


//
//        sleep(200);

        while(opModeIsActive()){
            drive.update();
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("odo", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("heading", Math.toDegrees(Angle.norm(drive.getRawExternalHeading())));
            telemetry.addData("target heading", Math.toDegrees(toHighPreload.end().getHeading()));

            telemetry.update();
        }

        lifter.finalLifterTicks = lifter.getCurrentPosition();
        intakeVechi.finalIntakeTicks = intakeVechi.getCurrentPosition();

        lifter.killLifterThread();
        intakeVechi.killIntakeThread();
    }

    TrajectorySequence toHighPreload() {
        return drive.trajectorySequenceBuilder(initialPoseLeft)

                .back(4)

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    lifter.setTargetTicks(1500, Lifter.LIFTER_LEVEL.HIGH.ticks);
                   intakeVechi.swingSetTargetTicks(0,100);

                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;

                    intakeVechi.spinFor(3000, 800, -0.6);
                })

                .build();
    }

    TrajectorySequence toConeOne(@NonNull TrajectorySequence toHighPreload) {
        return drive.trajectorySequenceBuilder(toHighPreload.end())
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(600, Lifter.LIFTER_LEVEL.LOW.ticks - 150);

                    lifter.setTargetTicks(1700, Lifter.LIFTER_LEVEL.LOW.ticks - 800);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    lifter.setTargetTicks(2550, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    intakeVechi.setCRServosPow(0);
                    intakeVechi.spinFor(0, 2700, -0.5);
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

                .lineToLinearHeading(new Pose2d(-45, -10, Math.toRadians(180)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(50))
                .setAccelConstraint(new ProfileAccelerationConstraint(40))

                .lineToConstantHeading(new Vector2d(-59.12, -10))

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
                    intakeVechi.spinFor(1600, 800, 0.8);
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

                .lineToLinearHeading(new Pose2d(-40, -9, Math.toRadians(180)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(-27.2, -2.2, Math.toRadians(228)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeTwo(@NonNull TrajectorySequence toHigh1) {
        return drive.trajectorySequenceBuilder(toHigh1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1500, Lifter.LIFTER_LEVEL.LOW.ticks - 900);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    lifter.setTargetTicks(2500, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
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

                .lineToLinearHeading(new Pose2d(-45, -10, Math.toRadians(180)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

                .lineToConstantHeading(new Vector2d(-59.10, -10))

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

                .lineToLinearHeading(new Pose2d(-40, -9, Math.toRadians(180)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(-27.2, -2.4, Math.toRadians(228)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeThree(@NonNull TrajectorySequence toHigh2) {
        return drive.trajectorySequenceBuilder(toHigh2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1400, Lifter.LIFTER_LEVEL.LOW.ticks - 1100);

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

                .lineToLinearHeading(new Pose2d(-45, -10, Math.toRadians(180)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

                .lineToConstantHeading(new Vector2d(-59.10, -10))

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

                .lineToLinearHeading(new Pose2d(-40, -9, Math.toRadians(180)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(-27.2, -2.5, Math.toRadians(229)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeFour(@NonNull TrajectorySequence toHigh3) {
        return drive.trajectorySequenceBuilder(toHigh3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intakeVechi.swingSetTargetTicks(100, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks - 50);
                    lifter.setTargetTicks(500, Lifter.LIFTER_LEVEL.LOW.ticks);

                    lifter.setTargetTicks(1400, Lifter.LIFTER_LEVEL.LOW.ticks - 1200);

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

                .lineToLinearHeading(new Pose2d(-45, -10, Math.toRadians(180)))

                .resetConstraints()

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

                .lineToConstantHeading(new Vector2d(-59.2, -10))

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
                    intakeVechi.spinFor(1600, 800, 0.8);
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

                .lineToLinearHeading(new Pose2d(-40, -9, Math.toRadians(180)))

                .setVelConstraint(new TranslationalVelocityConstraint(48))
                .setAccelConstraint(new ProfileAccelerationConstraint(43))

//                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))
                .lineToLinearHeading(new Pose2d(-27.2, -2.7, Math.toRadians(229)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toParkOne(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-59, -12, Math.toRadians(180)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(50)
                )
//                .lineTo(new Vector2d(-58, -13.5))

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
                .splineToSplineHeading(new Pose2d(-36.5, -15, Math.toRadians(270)), Math.toRadians(270),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(50)
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
                .splineToLinearHeading(new Pose2d(-11.5, -11.5, Math.toRadians(270)), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(50)
                )

                .build();
    }
}
