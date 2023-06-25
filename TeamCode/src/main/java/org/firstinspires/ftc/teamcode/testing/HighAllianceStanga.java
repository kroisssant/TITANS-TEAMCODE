package org.firstinspires.ftc.teamcode.testing;

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
public class HighAllianceStanga extends LinearOpMode {
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

    static Pose2d initialPoseLeft = new Pose2d(35, -63, Math.toRadians(90));

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

        TrajectorySequence toHigh = drive.trajectorySequenceBuilder(initialPoseLeft)
                .lineToConstantHeading(new Vector2d(12.5, -62))
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, IntakeVechi.INTAKE_STATE.OUTSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;
                    intakeVechi.spinFor(2000, 800, 0.6);
                })
                .splineToLinearHeading(new Pose2d(6, -32, Math.toRadians(125)), Math.toRadians(125))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(180)), Math.toRadians(180))
                //.lineToConstantHeading(new Vector2d(58, -35))
                .build();

        if (cameraOK) {
            while (target < 0 && timer.milliseconds() < 2000) {
                target = pipeline.targetFound;
            }
        }

        if (cameraOK) {
            telemetry.addLine("ok");
        } else {
            telemetry.addLine("perform reinitialization");
        }

        telemetry.update();


        //pipeline.stopAprilTagDetection();
        //webcam.stopStreaming();
        //webcam.closeCameraDevice();

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

        drive.setPoseEstimate(initialPoseLeft);

        drive.followTrajectorySequence(toHigh);

//        TrajectorySequence toPark1 = drive.trajectorySequenceBuilder(toHigh.end())
//                .setReversed(true)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
//                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//                })
//                .build();

        TrajectorySequence toPark2 = drive.trajectorySequenceBuilder(toHigh.end())
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })
                .lineToConstantHeading(new Vector2d(30, -35))
                .build();

        TrajectorySequence toPark3 = drive.trajectorySequenceBuilder(toHigh.end())
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;
                })
                .lineToConstantHeading(new Vector2d(58, -35))
                .build();

        sleep(1000);

        if (target == 0){

            automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, IntakeVechi.INTAKE_STATE.INSIDE);
            currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
            currentIntakeState = IntakeVechi.INTAKE_STATE.INSIDE;

        }
        else if(target == 2){
            drive.followTrajectorySequence(toPark3);
        }
        else {
            drive.followTrajectorySequence(toPark2);
        }

        while(!isStopRequested()){
            ;;
        }

        lifter.kill = true;
        intakeVechi.kill = true;

        lifter.killLifterThread();
        intakeVechi.killIntakeThread();

    }
}
