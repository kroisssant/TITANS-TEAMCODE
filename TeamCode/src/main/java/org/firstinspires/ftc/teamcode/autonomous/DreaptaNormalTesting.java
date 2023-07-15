package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Automatisms;
import org.firstinspires.ftc.teamcode.util.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class DreaptaNormalTesting extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(1.5F, new int[] {0, 1, 2});
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

    static Pose2d initialPoseRight = new Pose2d(36, -63, Math.toRadians(270));

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(600);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){

            @Override
            public void onOpened(){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                try{
                    Thread.sleep(1000); // always wait before pressing start
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode){
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

        initDetection();
        timer.reset();

        automatism = new Automatisms(lifter, intakeVechi);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = IntakeVechi.INTAKE_STATE.INIT;

        intakeVechi.spinFor(0, 50, 0.1); //bug

        if (cameraOK) {
            telemetry.addLine( "ok");
        } else {
            telemetry.addLine("perform reinitialization!");
        }

        telemetry.update();

        //pipeline.stopAprilTagDetection();
        //webcam.stopStreaming();
        //webcam.closeCameraDevice();

        TrajectorySequence toHighPreload = drive.trajectorySequenceBuilder(initialPoseRight)
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                })
                .setReversed(true)
                .lineTo(new Vector2d(34, -35))
                .splineToSplineHeading(new Pose2d(28.5, -2, Math.toRadians(320)), Math.toRadians(118))

                .build();

        TrajectorySequence toCone1 = drive.trajectorySequenceBuilder(toHighPreload.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
//                    intake.swingSetTargetTicks(0, 1000);
//                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
//                    intake.spinFor(2000, 800, -0.6);
//                    intake.swingSetTargetTicks(150, 1450);
//                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(52.35, -10, Math.toRadians(0)), Math.toRadians(0))

                .build();

        TrajectorySequence toHigh1 = drive.trajectorySequenceBuilder(toCone1.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(28.5, -1, Math.toRadians(319)), Math.toRadians(116))

                .build();

        TrajectorySequence toCone2 = drive.trajectorySequenceBuilder(toHigh1.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
//                    intake.swingSetTargetTicks(0, 1000);
//                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                    intake.swingSetTargetTicks(150, 1450);
//                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(52.35, -10, Math.toRadians(0)), Math.toRadians(0))

                .build();

        TrajectorySequence toHigh2 = drive.trajectorySequenceBuilder(toCone2.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(28.5, -1, Math.toRadians(319)), Math.toRadians(116))

                .build();


        TrajectorySequence toCone3 = drive.trajectorySequenceBuilder(toHigh2.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
//                    intake.swingSetTargetTicks(0, 1000);
//                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                    intake.swingSetTargetTicks(150, 1450);
//                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(52.35, -10, Math.toRadians(0)), Math.toRadians(0))

                .build();

        TrajectorySequence toHigh3 = drive.trajectorySequenceBuilder(toCone2.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(28.5, -1, Math.toRadians(318)), Math.toRadians(115))

                .build();

        TrajectorySequence toCone4 = drive.trajectorySequenceBuilder(toHigh3.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
//                    intake.swingSetTargetTicks(0, 1000);
//                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                    intake.swingSetTargetTicks(150, 1450);
//                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(52.35, -10, Math.toRadians(0)), Math.toRadians(0))

                .build();

        TrajectorySequence toHigh4 = drive.trajectorySequenceBuilder(toCone4.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(28.5, -1, Math.toRadians(319)), Math.toRadians(116))

                .build();

        TrajectorySequence toCone5 = drive.trajectorySequenceBuilder(toHigh4.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
//                    intake.swingSetTargetTicks(0, 1000);
//                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                    intake.swingSetTargetTicks(150, 1450);
//                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(52.35, -9.5, Math.toRadians(0)), Math.toRadians(0))

                .build();

        TrajectorySequence toHigh5 = drive.trajectorySequenceBuilder(toCone5.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                    intake.spinFor(2000, 800, 0.6);
//                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(28.5, -0.88, Math.toRadians(318)), Math.toRadians(117))

                .build();

        TrajectorySequence toPark1 = drive.trajectorySequenceBuilder(toHigh5.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                })
                .splineToSplineHeading(new Pose2d(32, -12, Math.toRadians(270)), Math.toRadians(270))
                .lineTo(new Vector2d(12, -11))

                .setReversed(false)

                .build();

        TrajectorySequence toPark2 = drive.trajectorySequenceBuilder(toHigh5.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(35, -17, Math.toRadians(270)), Math.toRadians(270))

                .build();

        TrajectorySequence toPark3 = drive.trajectorySequenceBuilder(toHigh3.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40, -10, Math.toRadians(0)), Math.toRadians(0))

                .build();

        waitForStart();

        if (cameraOK) {
            pipeline.startAprilTagDetection();
            timer.reset();

            while (target < 0 && timer.milliseconds() < 1500){
                target = pipeline.targetFound;
            }
        }

        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();


        telemetry.addData("target:", target);
        telemetry.update();

        intakeThread = new Thread(intakeVechi);
        lifterThread = new Thread(lifter);

        lifterThread.start();
        intakeThread.start();

        drive.setPoseEstimate(initialPoseRight);

        drive.followTrajectorySequence(toHighPreload);
        sleep(50);

//        drive.followTrajectorySequence(toCone1);
//        sleep(50);
//        drive.followTrajectorySequence(toHigh1);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone2);
//        sleep(50);
//        drive.followTrajectorySequence(toHigh2);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone3);
//        sleep(50);
//        drive.followTrajectorySequence(toHigh3);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone4);
//        sleep(50);
//        drive.followTrajectorySequence(toHigh4);
//        sleep(50);
//
//        if(timer.seconds() >= 4){
//            drive.followTrajectorySequence(toCone5);
//            sleep(50);
//            drive.followTrajectorySequence(toHigh5);
//            sleep(50);
//        }
//
//        if (target == 0){
//            drive.followTrajectorySequence(toPark1);
//        }
//        else if(target == 2){
//            drive.followTrajectorySequence(toPark3);
//        }
//        else {
//            drive.followTrajectorySequence(toPark2);
//        }

        lifter.kill = true;
        intakeVechi.kill = true;

    }
}
