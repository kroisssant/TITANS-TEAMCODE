package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Automatisms;

@TeleOp
@Disabled
public class Drive extends LinearOpMode {

    public SampleMecanumDrive drive;
    private ControllerInput controller1, controller2;

    private Automatisms automatism;

    public Lifter lifter;
    public IntakeVechi intake;

    public Lifter.LIFTER_LEVEL currentLifterState;
    public IntakeVechi.INTAKE_STATE currentIntakeState;

    protected Thread intakeThread, lifterThread;

    public static volatile boolean here = false;

    private enum DRIVE_MODE{
        MANUAL, //driver controls everything
        AUTO    //uses automatisms
    }

    private DRIVE_MODE driveMode;

    public boolean paConIsInside = true;

    private void handleManualControl(ControllerInput _controller){
        lifter.manual = true;

        if(_controller.rightBumperOnce()){
            lifter.setManualPower(-0.2);
        }

        if(_controller.leftBumperOnce()){
            intake.swingSetTargetTicks(0, -100);
        }
    }


    private void handleAutomizedControl(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        //intake out (swing)


        if(_controller.dpadUp()){
            intake.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks);
        } else if (_controller.dpadDown()){
            intake.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
        }

        //low
        if (_controller.circleOnce()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.LOW.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.LOW;
        }

        //down cu intake unde era
        if(_controller.leftBumper()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.DOWN.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        }

        //poz de colectare
        if (_controller.squareOnce()) {
            if (currentIntakeState == IntakeVechi.INTAKE_STATE.OUTSIDE){
                intake.swingSetTargetTicks(0, 1200);
            } else {
                intake.swingSetTargetTicks(0, 30);
            }
        }

        if (_controller.crossOnce()) {
            lifter.setTargetTicks(0, 800);
            currentLifterState = Lifter.LIFTER_LEVEL.LOW;
        }

        //cone in/out while pressing
        //suge/scuipa
        if (_controller.left_trigger > 0.5) {
            intake.setCRServosPow(-0.8);
        } else if (_controller.right_trigger > 0.5) {
            intake.setCRServosPow(0.8);
        } else {
            intake.setCRServosPow(0.0);
        }

        //high intake unde era
         if(_controller.rightBumper()){
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.HIGH.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
        }

        if (_controller.triangleOnce()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.MID.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.MID;
        }
    }

    private void handleDriving(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -_controller.left_stick_y,
                        -_controller.left_stick_x,
                        -_controller.right_stick_x
                )
        );
        if (_controller.circleOnce() && paConIsInside) {
            paConIsInside = false;
            intake.paConOutside(0);
        } else
        if (_controller.circleOnce() && !paConIsInside) {
            paConIsInside = true;
            intake.paConInside(0);
        }

        //TODO !!!!
//        if(_controller.rightBumper() && _controller.leftBumper()){
//            lifter.manual = true;
//            lifter.setManualPower(-0.1);
//        }
//        else{
//            lifter.manual = false;
//            lifter.setTargetTicks(0, lifter.getCurrentPosition());
//        }

        drive.update();
    }

    private void handleDrivingSlowed(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.setWeightedDrivePower(new Pose2d(
                -_controller.left_stick_y * 0.25,
                -_controller.left_stick_x * 0.25,
                -_controller.right_stick_x * 0.25
        ));

//        if (_controller.circleOnce() && paConIsInside) {
//            paConIsInside = false;
//            intake.paConOutside(0);
//        } else
//        if (_controller.circleOnce() && !paConIsInside) {
//            paConIsInside = true;
//            intake.paConInside(0);
//        }

        drive.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        driveMode = DRIVE_MODE.AUTO;

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new IntakeVechi(hardwareMap, true);
        intake.telemetry = telemetry;

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        automatism = new Automatisms(lifter, intake);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = IntakeVechi.INTAKE_STATE.INIT;

        intake.spinFor(0,50, 0.1); //bug

        waitForStart();

        lifter.manual = false; //paranoia - de cand liftul se ridica singur

        intakeThread = new Thread(intake);
        lifterThread = new Thread(lifter);

        intakeThread.start();
        lifterThread.start();


        while(opModeIsActive()){

            controller1.update();
            controller2.update();
//            telemetry.addData("here", here);
//            telemetry.addData("mode", driveMode);
//            telemetry.update();


            //controller 2 - mechanisms n stuff
            switch(driveMode){
                case MANUAL: {
                    handleManualControl(controller2);

                    if(controller2.shareOnce()){
                        driveMode = DRIVE_MODE.AUTO;
                    }

                    break;
                }
                case AUTO: { //rn stam doar pe cazul auto
                   // lifter.manual = false;
                    handleAutomizedControl(controller2);

                    if(controller2.shareOnce()){
                        driveMode = DRIVE_MODE.MANUAL;
                    }

                    break;
                }
            }

            // controller 1 - chassis + ia lasa con
            if (controller1.rightBumper()) {
                handleDrivingSlowed(controller1);
            } else {
                handleDriving(controller1);
            }
        }


//        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
//        currentIntakeState = Intake.INTAKE_STATE.INIT;

        lifter.kill = true;
        intake.kill = true;

//        intake.killIntakeThread();
//        lifter.killLifterThread();
//        driveMode = DRIVE_MODE.AUTO;

    }
}

