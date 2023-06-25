package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNou;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Automatisms;
@TeleOp(name = "Duck u dimi", group = "PP Rebuild")
public class DrivePtKidsFestDuckYouDimi extends LinearOpMode {

    public ServoImplEx servo180;
    public ServoImplEx claw;
    public ServoImplEx servoExtins;
    public DcMotorEx motorIntake;

    public SampleMecanumDrive drive;
    private ControllerInput controller1, controller2;

    public Lifter lifter;

    public Thread lifterThread;

    public volatile boolean isMotorIntakeMoving = false;

    public static volatile boolean here = false;

    private enum DRIVE_MODE{
        MANUAL, //driver controlls everything
        AUTO,   //uses automatisms
        HYBRID
        //auto currently disabled
        //manual currently disabled
    }

    private DRIVE_MODE driveMode;

    /*private void handleManualControl(ControllerInput _controller){

    }*/


    /*private void handleAutomizedControl(ControllerInput _controller) {

    }*/

    private int deg = 10;
    private boolean lateralGrab = false;
    private double sliderPos = 1d;
    private final double kSlide = .0005;

    private void handleHybridControl(ControllerInput _controller) {
        if(_controller.options()){ // skip if switching player
            return;
        }
        if(_controller.dpadUpOnce()) {
//            intake.setTargetDeg(0, 170); //TODO
                motorIntake.setTargetPosition(470);
                motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorIntake.setPower(0.4);
        } else if(_controller.dpadDownOnce()) {
//            intake.setTargetDeg(0, 0);
            motorIntake.setTargetPosition(10);
            motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorIntake.setPower(0.4);
        }
        // toggle claw
        // left trigger once is a new function in ControllerInput
        // if it doesnt work change it for something else
        if(_controller.left_trigger > 0.3){
            claw.setPosition(0);
        }
        if(_controller.right_trigger > 0.3) {
            claw.setPosition(0.4);
        }
        // go to high - lifter
        // previous high was at 2700 but its not really useful anymore
        if(_controller.rightBumperOnce()){
            lifter.setTargetTicks(0, 1800);
        }

        // go to mid - lifter
        // previous mid was 1800
        if(_controller.triangleOnce()){
            lifter.setTargetTicks(0, 1000);
        }

        // go to down - lifter
        // init is at 0 but that position lets the claw hang dangerously close to the tiles
        if(_controller.circleOnce()){
            lifter.setTargetTicks(0, 500);
        }

        // toggle 180 - 10 deg intake
//        if(_controller.circleOnce()){
//            if(deg < 100){
//                deg = 180;
//            } else {
//                deg = 10;
//            }
//
//            intake.setTargetDeg(0, deg);
//        }

        // note that after a lateral grab one can rotate the intake to stack cones
        // on deg > 100 lateral grab clears automatically
//        if(intake.getCurrentDeg() < 100 || intake.getCurrentDeg() > 300){
//            if(!lateralGrab){
//                intake.rotateClawToAngle(0, 180);
//            }
//        } else if(intake.getCurrentDeg() > 100){
//            intake.rotateClawToAngle(0, 0);
//            lateralGrab = false;
//            telemetry.addLine("jasjdl");
//        }

        // lateral grab - might work on fallen down cones
//        if(_controller.leftBumperOnce()){
//            if(! lateralGrab){
//                if(deg < 50) { // lateral grab has no purpose in the air, so put this to prevent accidental presses
//                    intake.rotateClawToAngle(0, 90);
//                    lateralGrab = true;
//                }
//            } else {
//                lateralGrab = false;
//                intake.rotateClawToAngle(0, 0);
//            }
//        }

        if(_controller.leftBumperOnce()){
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.DOWN.ticks);
        }
    }

        // extend slider slowly with right_stick_x
        // retract slider quickly - min legth

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
        drive.update();
    }

    private void handleDrivingSlowed(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.setWeightedDrivePower(new Pose2d(
                -_controller.left_stick_y * 0.35,
                -_controller.left_stick_x * 0.35,
                -_controller.right_stick_x * 0.35
        ));

        drive.update();
    }

    private void loopThroughDrivemodes(){
        if(driveMode == DRIVE_MODE.HYBRID){
            return;
            //driveMode = DRIVE_MODE.AUTO;
        } else if (driveMode == DRIVE_MODE.AUTO){
            driveMode = DRIVE_MODE.MANUAL;
        } else {
            driveMode = DRIVE_MODE.HYBRID;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // ------------- INIT ----------------
        driveMode = DRIVE_MODE.HYBRID;

        servo180 = hardwareMap.get(ServoImplEx.class, "servo180");
        servo180.scaleRange(.0, 0.66);
        servo180.setPosition(1);

        claw = hardwareMap.get(ServoImplEx.class, "claw");

        // slider
        servoExtins = hardwareMap.get(ServoImplEx.class, "servoExtins");
        servoExtins.setPwmEnable();
        servoExtins.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servoExtins.setDirection(Servo.Direction.FORWARD);
        servoExtins.scaleRange(.4, 1d);
        servoExtins.setPosition(1);

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        telemetry.addLine("mechanisms initialized. press START");
        telemetry.update();

        lifterThread = new Thread(lifter);

        lifterThread.start();

        lifter.setTargetTicks(0, 300);
        // -------------- START --------------
        waitForStart();

//        intake.startIntakeThread();
//        lifter.startLifterThread();
//        intake.endClawProcessesForcibly();

        // intake.servo180.setPosition(1);

        // claw will push too much into the floor otherwise
        // no need to wait until these finish since newer targets take priority in the run() functions
        lifter.setTargetTicks(0, 200);
//        intake.setTargetDeg(0, 15);

        // ------------- DRIVE --------------
        while(opModeIsActive()){
            controller1.update();
            controller2.update();

            telemetry.addData("mode", driveMode);
            telemetry.addData("lateralGrab", lateralGrab);
            telemetry.update();

            // controller 2
            switch(driveMode){
                case MANUAL: {
                    break;
                }
                case AUTO: {
                    break;
                }
                case HYBRID: {
                    handleHybridControl(controller2);
                    break;
                }
            }

            // change drive modes
            if(controller2.shareOnce()){
                loopThroughDrivemodes();
            }

            // controller 1
            if (controller1.rightBumper()) {
                handleDrivingSlowed(controller1);
            } else {
                handleDriving(controller1);
            }

            if(lifter.getCurrentPosition() > 3000){
                lifter.killLifterThread();
                telemetry.setAutoClear(false);
                telemetry.addLine("Lifter went too high unexpectedly. Thread killed. STOP and REINITIALIZE");
                telemetry.update();
            }
        }

        // --------------- STOP ----------------
//        intake.endClawProcessesForcibly();
        lifter.killLifterThread();
    }
}

