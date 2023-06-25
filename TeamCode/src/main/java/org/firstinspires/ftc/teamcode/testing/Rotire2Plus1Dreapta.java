package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Disabled
@Autonomous
public class Rotire2Plus1Dreapta extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(2, new int[] {0, 1, 2});
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

    static Pose2d initialPoseRight = new Pose2d(36, -63, Math.toRadians(90));

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000);
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

        TrajectorySequence toHighPreload = drive.trajectorySequenceBuilder(initialPoseRight)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intakeVechi.swingSetTargetTicks(200, -100);
                })
                //to high preload
                .forward(30)
                .splineToSplineHeading(new Pose2d(49, -12, Math.toRadians(180 - 180)), Math.toRadians(180 - 170))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                    intakeVechi.spinFor(2000, 800, 0.6);
                })
                .splineToLinearHeading(new Pose2d(-28, -1, Math.toRadians(180 - 235)), Math.toRadians(180 - 45))
                // --
                .build();

        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(toHighPreload.end())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
                    lifter.setTargetTicks(750, 550);
                    intakeVechi.swingSetTargetTicks(0, 1000);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;

                    intakeVechi.spinFor(1600, 1800, -0.6);
                    intakeVechi.swingSetTargetTicks(0, 2000);

                    lifter.setTargetTicks(2500, 1900);
                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                })
                .splineToSplineHeading(new Pose2d(59.8, -12, Math.toRadians(180 - 180)), Math.toRadians(180 - 195))

                .build();

        TrajectorySequence toHighFromStack1 = drive.trajectorySequenceBuilder(toStack1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;
                    intakeVechi.spinFor(2000, 600, 0.6);
                })

                .setReversed(true)
                .back(20)
                .splineToSplineHeading(new Pose2d(-28, 0, Math.toRadians(270 - 47)), Math.toRadians(65))

                .build();


        if (cameraOK) {
            telemetry.addLine( "ok");
        } else {
            telemetry.addLine("perform reinitialization!");
        }

        telemetry.update();

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
        sleep(1000);
        drive.followTrajectorySequence(toStack1);
        sleep(1600);
        drive.followTrajectorySequence(toHighFromStack1);
        sleep(1000);
        drive.followTrajectorySequence(toStack1);
        sleep(1600);
        drive.followTrajectorySequence(toHighFromStack1);
        sleep(1000);

        TrajectorySequence toPark1 = drive.trajectorySequenceBuilder(toHighFromStack1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-57, -12, Math.toRadians(180)), Math.toRadians(195))
                .build();

        TrajectorySequence toPark2 = drive.trajectorySequenceBuilder(toHighFromStack1.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })
                .splineToSplineHeading(new Pose2d(-37, -25, Math.toRadians(-90)), Math.toRadians(270))

                .build();

        TrajectorySequence toPark3 = drive.trajectorySequenceBuilder(toHighFromStack1.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })
//                .splineToLinearHeading(new Pose2d(-15, -13, Math.toRadians(180)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-13, -15, Math.toRadians(180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-13, -13.5, Math.toRadians(0)), Math.toRadians(-180))

                .build();
        if (target == 0){
            drive.followTrajectorySequence(toPark1);
        }
        else if(target == 2){
            drive.followTrajectorySequence(toPark3);
        }
        else {
            drive.followTrajectorySequence(toPark2);
        }

        lifter.kill = true;
        intakeVechi.kill = true;


    }
}
