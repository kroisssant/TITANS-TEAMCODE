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
public class autoTaranesc extends LinearOpMode {
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

    static Pose2d initialPoseLeft = new Pose2d(-36, -63, Math.toRadians(90));

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
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, IntakeVechi.INTAKE_STATE.OUTSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = IntakeVechi.INTAKE_STATE.OUTSIDE;
                    intakeVechi.spinFor(2000, 800, 0.6);
                })
                .forward(30)
                .splineToSplineHeading(new Pose2d(-28.5, -8.5, Math.toRadians(62)), Math.toRadians(62))

                .build();

        if (cameraOK) {
            telemetry.addLine( "ok");
        } else {
            telemetry.addLine("perform reinitialization!");
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


        TrajectorySequence front = drive.trajectorySequenceBuilder(initialPoseLeft)
                .forward(70)
                .build();

        TrajectorySequence toPark1 = drive.trajectorySequenceBuilder(front.end())
                .back(20)
                .strafeLeft(22)
                .build();

        TrajectorySequence toPark2 = drive.trajectorySequenceBuilder(front.end())
                .back(20)
                .build();

        TrajectorySequence toPark3 = drive.trajectorySequenceBuilder(front.end())
                .back(20)
                .strafeRight(22)
                .build();

        drive.followTrajectorySequence(front);

        sleep(15000);

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
