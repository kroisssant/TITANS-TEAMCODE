package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNou;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNouV2;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.mechanisms.LifterNou;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@TeleOp
public class teleop extends LinearOpMode {
    SampleMecanumDrive drive;
    LifterNou lifter;
    IntakeNouV2 intake;
    private ControllerInput controller1, controller2;
    private boolean flagGoingFrontIntake = true;

    private int decDistance = 100;
    private int decDistanceIntake = 100;
    private double intakePower = 0;

    private boolean flagTwist = false;
    private boolean flagGuide = false;
    private boolean flagExtendo = false;
    private boolean flagClaw = false;
    private boolean flagUp = false;

    LifterNou.LIFTER_LEVEL lastState = null;
    LifterNou.LIFTER_LEVEL robotState = LifterNou.LIFTER_LEVEL.DOWN;

    LifterNou.LIFTER_LEVEL reqState = null;

    double power = 0.9, powerOnDec = 0.25;
    boolean cycleSolo = false;
    boolean lifterSlow = false;
    @Override
    public void runOpMode() throws InterruptedException {
        lifter = new LifterNou(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeNouV2(hardwareMap, telemetry);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        lifter.setPosition(50);
        lifter.setPower(0.3, 0, 0);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            controller1.update();
            controller2.update();

            if (controller1.options()) { // skip if switching player
                return;
            }

            if (controller2.options()) { // skip if switching player
                return;
            }

            if (controller1.rightBumper() ) {
                drive.setWeightedDrivePower(new Pose2d(   //SLOW
                        controller1.left_stick_y * 0.25 +0.00001,
                        controller1.left_stick_x * 0.3,
                        -controller1.right_stick_x * 0.35
                ));
            }
            else {
                drive.setWeightedDrivePower(new Pose2d(     //REGULAR
                        controller1.left_stick_y * 0.7 +0.00001,
                        controller1.left_stick_x * 0.45,
                        -controller1.right_stick_x * 0.35
                ));
            }

            if((controller1.leftBumperOnce() && flagClaw ==false) ){
                intake.claw.setPosition(0.48);
                flagClaw =  true;
            }
            else if((controller1.leftBumperOnce() && flagClaw == true) ) {
                intake.claw.setPosition(0.3);
                flagClaw = false;
            }

            mechanism();
            drive.update();

            lifter.copyBehaviour();
            lifter.setPower(power, powerOnDec, decDistance);
          //  lifter.checkHeight();
//            if(lifterSlow){
//                lifter.setPower(power, powerOnDec, decDistance);
//            }

            intake.bratMovement(intakePower, decDistanceIntake);
        }

    }

    public void mechanism() {
//        // cycle solo - not needed
//        if(controller2.dpadRightOnce() && cycleSolo == false){
//            cycleSolo = true;
//        }
//        else if(controller2.dpadRightOnce() && cycleSolo == true){
//            cycleSolo = false;
//        }

        //right bumper - claw open
        if(controller2.rightBumperOnce() && flagClaw == true && flagUp == true){
            intake.claw.setPosition(0.3);
            flagClaw = false;
        }

        //dpad down - guide manual
        if(controller2.dpadDownOnce()){
            intake.guide.setPosition((flagGuide)?(1d):(.75d));
            flagGuide = !flagGuide;
        }

        // left trigger - ground
        if(controller2.left_trigger > 0.3) {
            lifter.setPosition(LifterNou.LIFTER_LEVEL.DOWN.ticks);

            lifterSlow = true;

            decDistance = 100;
            //decDistance = lifter.decelerationDistanceCalculator();

            flagUp = false;
            flagGuide = false;

            intake.guide.setPosition(1d);

            intake.bratSetPosition(125);
            intakePower = 0.6;
            decDistanceIntake = 100;

            intake.servo180.setPosition(1);

            intake.servoExtins.setPosition(1);

            lastState = robotState;
            robotState = LifterNou.LIFTER_LEVEL.DOWN;
        }

        // left bumper - low
        if(controller2.leftBumperOnce()) {
            lifter.setPosition(LifterNou.LIFTER_LEVEL.LOW.ticks);
            lifterSlow = true;


            decDistance = 100;
            //decDistance = lifter.decelerationDistanceCalculator();

            flagUp = true;

            intake.bratSetPosition(430);
            intakePower = 0.35;
            decDistanceIntake = 100;

            intake.servo180.setPosition(1);

            intake.servoExtins.setPosition(1);

            lastState = robotState;
            robotState = LifterNou.LIFTER_LEVEL.LOW;

        }

        // right bumper - high
        if(controller2.rightBumperOnce()) {
            lifter.setPosition(LifterNou.LIFTER_LEVEL.HIGH.ticks);

            lifterSlow = true;
            lastState = robotState;
            robotState = LifterNou.LIFTER_LEVEL.HIGH;
            decDistance = 100;
            // decDistance = lifter.decelerationDistanceCalculator();

            flagUp = true;

            intake.bratSetPosition(1400);
            intakePower = 0.72;
            decDistanceIntake = 100;

            intake.servoExtins.setPosition(1);

            intake.servo180.setPosition(0);
        }

        //triangle / Y - mid

        if(controller2.triangleOnce()) {
            lifter.setPosition(LifterNou.LIFTER_LEVEL.MID.ticks);

            lifterSlow = true;
            lastState = robotState;
            robotState = LifterNou.LIFTER_LEVEL.MID;
            decDistance = 100;
            //decDistance = lifter.decelerationDistanceCalculator();

            flagUp = true;

            intake.bratSetPosition(1450);
            intakePower = 0.77;
            decDistanceIntake = 100;

            intake.servo180.setPosition(0);

            intake.servoExtins.setPosition(1);

        }

        // manual lifter - leftStickY
        else if(controller2.left_stick_y < -0.3 || controller2.left_stick_y > 0.3){
            lifterSlow = false;
            decDistance = 0;
            lifter.setPosition((int) (lifter.getPosition() + controller2.left_stick_y * 100));

            flagUp = true;
        }

        //con picat
        if(controller2.squareOnce()){
            lifter.setPosition(400);
            decDistance = 100; //??

            intake.bratSetPosition(60);
            intakePower = 0.25;
            decDistanceIntake = 100;

            intake.servo180.setPosition(0);
        }

        //right stick y - manual brat up and down
        if(controller2.right_stick_y < -0.3 || controller2.right_stick_y > 0.3){
            decDistanceIntake = 0;
            intake.bratSetPosition((int)(intake.leftMotorIntake.getCurrentPosition() + controller2.right_stick_y * 100));
            intakePower = 0.25;
        }

        //twist manual
        if(controller2.circleOnce() && flagTwist == false){
            intake.servo180.setPosition(1);
            flagTwist =  true;
        }
        else if(controller2.circleOnce() && flagTwist == true) {
            intake.servo180.setPosition(0);
            flagTwist = false;
        }

//        if(!lifter.leftLifter.isBusy()) {
//            lifterSlow = false;
//        }
    }
}
