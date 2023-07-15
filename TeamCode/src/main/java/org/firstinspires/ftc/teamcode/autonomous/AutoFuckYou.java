package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.ControllerInput;
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
import java.util.Random;


import java.util.Arrays;

@SuppressWarnings("all")
@Autonomous  (name = "Fuck You - auto stanga", group="PP Rebuild")
public class AutoFuckYou extends LinearOpMode {

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

    public ServoImplEx servo180;

    public ServoImplEx guide;

    public ServoImplEx claw;
    public ServoImplEx servoExtins;
    public DcMotorEx motorIntake;

    private ControllerInput controller1, controller2;


    public Thread lifterThread;

    public volatile boolean isMotorIntakeMoving = false;

    public static volatile boolean here = false;
    public final Pose2d initialPoseStanga = new Pose2d(0, 0, Math.toRadians(0));

    boolean flag = false;
    boolean flagTwist = false;
    boolean flagExtendo = false;

    public TrajectorySequence toStaticAutoPose(Pose2d initialPose){
        return drive.trajectorySequenceBuilder(initialPose)
                .lineToConstantHeading(new Vector2d(-50, 2),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60 ,DriveConstants.TRACK_WIDTH)
                                )
                        ),new ProfileAccelerationConstraint(30)
                )
                .turn(Math.toRadians(81))
                .lineToConstantHeading(new Vector2d(-48.5, -20.5))
                .addTemporalMarker(2,() ->{
                    brat(420, 0.35);
                })
                .build();
    }

    /*
    x
    |
    |
    |
    |
    O----------- y
    */

    public TrajectorySequence toPark(Pose2d initialPose, int target){
        if(target == -1){
            Random rand = new Random();
            target = rand.nextInt(3);
            telemetry.addData("Random parking generated", target);
            telemetry.update();
        }

        double y = (target == 0)?(-23.5):((target == 1)?(2):(25));
        return drive.trajectorySequenceBuilder(new Pose2d(initialPose.getX(), initialPose.getY(), Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-54, y, Math.toRadians(90)), new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
                        )
                ), new ProfileAccelerationConstraint(40))
                .build();
    }


//    Trajectory traj = drive.trajectoryBuilder(new Pose2d(0,0,0))
//            .lineToConstantHeading(new Vector2d(-35, -12),
//                    new MinVelocityConstraint(
//                            Arrays.asList(
//                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                    new MecanumVelocityConstraint(100 ,DriveConstants.TRACK_WIDTH)
//                            )
//                    ),new ProfileAccelerationConstraint(100)
//            )
//            .turn(Math.toRadians(80))
//            .lineToConstantHeading(new Vector2d(-55, -12))
//            .build();


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

    private  void brat( int target, double power){

        motorIntake.setTargetPosition(target);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(power);

    }
    private void initHardware() {





        servo180 = hardwareMap.get(ServoImplEx.class, "servo180");
        servo180.setDirection(Servo.Direction.REVERSE);
        servo180.scaleRange(.03, 0.7);
        servo180.setPosition(1);

        claw = hardwareMap.get(ServoImplEx.class, "claw");

        // slider
        servoExtins = hardwareMap.get(ServoImplEx.class, "servoExtins");
        servoExtins.setPosition(1);

        guide = hardwareMap.get(ServoImplEx.class, "guide");
        guide.setDirection(Servo.Direction.REVERSE);
        guide.setPosition(1);


        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
    }




        @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);

        initDetection();
        initHardware();
        timer.reset();

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        TrajectorySequence toStaticAuto = toStaticAutoPose(initialPoseStanga);


        pipeline.startAprilTagDetection();

        while(!isStarted() && !isStopRequested()){
            if(!cameraOK){
                telemetry.addLine("Camera failed to start. Please RESTART");
            } else {
                telemetry.addLine("Camera is OK. Waiting...");
            }
            if(pipeline.targetFound != -1){
                telemetry.addLine("Everything alright. Press START");
            }

            telemetry.update();
        }

        waitForStart();
        telemetry.setAutoClear(false);


        target = pipeline.targetFound;

        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        telemetry.addData("Sleeve", target + 1);
        telemetry.update();

        lifterThread = new Thread(lifter);

        lifterThread.start();
        TrajectorySequence toPark = toPark(toStaticAuto.end(), target);

            lifter.setTargetTicks(0, 0);
            brat(130, 0.25);
            servo180.setPosition(1);
            sleep(100);
            claw.setPosition(0.5);
            sleep(100);

        drive.followTrajectorySequence(toStaticAuto);
//        sleep(500);


            //TODO:PRELOAD
            lifter.setTargetTicks(0, 1000);
            guide.setPosition(0.65);
            servo180.setPosition(0);
            servoExtins.setPosition(1);

            brat(1480, 0.9);
            sleep(550);
            brat(1520, 0.25);
            sleep(500);

            servoExtins.setPosition(0.55);
            sleep(550);
            brat(1800, 0.6);
            sleep(300);
            claw.setPosition(0.1);
            guide.setPosition(0.9);
            servoExtins.setPosition(1);
            sleep(100);


          //TODO: SERIA 1
        lifter.setTargetTicks(0, 300);
        servoExtins.setPosition(1);

        brat(195, 0.8);
        sleep(100);
            servo180.setPosition(1);
            sleep(500);
        brat(245, 0.25);
        servo180.setPosition(1);

        sleep(650);
        servoExtins.setPosition(0.76);
        sleep(350);
        claw.setPosition(0.5);
        sleep(150);
        brat(470, 0.25);
        lifter.setTargetTicks(0, 1050);
        guide.setPosition(0.65);
        servo180.setPosition(0);
        servoExtins.setPosition(1);
        sleep(100);
            brat(1480, 0.9);
            sleep(550);
            brat(1520, 0.25);
            sleep(500);
        servoExtins.setPosition(0.55);
        sleep(550);
        brat(1800, 0.4);
        sleep(200);
        claw.setPosition(0.1);
        guide.setPosition(0.9);
        servoExtins.setPosition(1);
        sleep(150);



            //TODO: SERIA 2
            servoExtins.setPosition(1);
            lifter.setTargetTicks(0, 0);
            brat(260, 0.8);
            claw.setPosition(0.1);
            sleep(100);
            servo180.setPosition(1);
            sleep(500);
            brat(260, 0.25);
            servo180.setPosition(1);
            claw.setPosition(0.1);
            sleep(600);
            servoExtins.setPosition(0.75);
            sleep(400);
            claw.setPosition(0.5);
            sleep(150);
            brat(470, 0.25);
            lifter.setTargetTicks(0, 1070);
            guide.setPosition(0.65);
            servo180.setPosition(0);
            servoExtins.setPosition(1);
            sleep(100);
                brat(1480, 0.9);
                sleep(550);
                brat(1520, 0.25);
                sleep(500);
            servoExtins.setPosition(0.55);
            sleep(550);
            brat(1800, 0.4);
            sleep(200);
            claw.setPosition(0.05);
            guide.setPosition(0.9);
            servoExtins.setPosition(1);
            sleep(150);




            //TODO: SERIA 3
            servoExtins.setPosition(1);
            lifter.setTargetTicks(0, 0);
            brat(230, 0.8);
            sleep(100);
            servo180.setPosition(1);
            sleep(550);
            brat(230, 0.25);
            servo180.setPosition(1);
            claw.setPosition(0.1);
            sleep(600);
            servoExtins.setPosition(0.75);
            sleep(400);
            claw.setPosition(0.5);
            sleep(150);
            brat(470, 0.25);
            lifter.setTargetTicks(0, 1070);
            guide.setPosition(0.65);
            servo180.setPosition(0);
            servoExtins.setPosition(1);
            sleep(100);
                brat(1480, 0.9);
                sleep(550);
                brat(1520, 0.25);
                sleep(500);
            servoExtins.setPosition(0.55);
            sleep(550);
            brat(1800, 0.4);
            sleep(200);
            claw.setPosition(0.05);
            guide.setPosition(0.9);
            servoExtins.setPosition(1);
            sleep(100);



            //TODO: SERIA 4
            servoExtins.setPosition(1);
            lifter.setTargetTicks(0, 0);
            brat(220, 0.8);
            sleep(100);
            servo180.setPosition(1);
            sleep(550);
            brat(220, 0.25);
            servo180.setPosition(1);
            claw.setPosition(0.1);
            sleep(500);
            servoExtins.setPosition(0.75);
            sleep(400);
            claw.setPosition(0.5);
            sleep(150);
            brat(470, 0.25);
            lifter.setTargetTicks(0, 1070);
            guide.setPosition(0.65);
            servo180.setPosition(0);
            servoExtins.setPosition(1);
            sleep(100);
            brat(1480, 0.9);
            sleep(550);
            brat(1520, 0.25);
            sleep(500);
            servoExtins.setPosition(0.55);
            sleep(550);
            brat(1800, 0.4);
            sleep(200);
            claw.setPosition(0.05);
            guide.setPosition(0.9);
            servoExtins.setPosition(1);
            sleep(100);


            //TODO: SERIA 5
            servoExtins.setPosition(1);
            lifter.setTargetTicks(0, 0);
            brat(190, 0.8);
            sleep(100);
            servo180.setPosition(1);
            sleep(550);
            brat(190, 0.25);
            servo180.setPosition(1);
            claw.setPosition(0.1);
            sleep(500);
            servoExtins.setPosition(0.75);
            sleep(400);
            claw.setPosition(0.5);
            sleep(150);
            brat(470, 0.25);
            lifter.setTargetTicks(0, 1030);
            guide.setPosition(0.65);
            servo180.setPosition(0);
            servoExtins.setPosition(1);
            sleep(100);
            brat(1480, 0.9);
            sleep(550);
            brat(1520, 0.25);
            sleep(500);
            servoExtins.setPosition(0.55);
            sleep(550);
            brat(1750, 0.4);
            sleep(300);
            claw.setPosition(0.05);
            guide.setPosition(0.9);
            servoExtins.setPosition(1);
            sleep(100);



            //TODO: PARK
            lifter.setTargetTicks(0, 0);
            brat(280, 0.8);
            sleep(100);
            servo180.setPosition(1);
            sleep(400);
            brat(280, 0.25);
            claw.setPosition(0.1);

//            sleep(500);
            drive.followTrajectorySequence(toPark);

            lifter.kill = true;

    }

}
