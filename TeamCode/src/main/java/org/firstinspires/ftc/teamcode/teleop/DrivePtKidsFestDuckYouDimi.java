package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNou;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.mechanisms.LifterNou;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Automatisms;
@SuppressWarnings("all")
@TeleOp(name = "Duck u dimi", group = "PP Rebuild")
public class DrivePtKidsFestDuckYouDimi extends LinearOpMode {

    public ServoImplEx servo180;
    public ServoImplEx claw;
    public ServoImplEx servoExtins;
    public ServoImplEx guide;
    public DcMotorEx motorIntake;
    public DcMotorEx motorIntake2;

    public SampleMecanumDrive drive;
    private ControllerInput controller1, controller2;

    public LifterNou lifter;
    double power, powerOnDec;
    int decDistance = 0;


    public Thread lifterThread;

    public volatile boolean isMotorIntakeMoving = false;

    public static volatile boolean here = false;

    boolean flag = false;

    boolean flagUp = false ;
    boolean flagTwist = false;
    boolean flagExtendo = false;
    boolean flagGuide = false;

    boolean flagSlow = false;

    boolean cycleSolo = false;


    private ElapsedTime runtime = new ElapsedTime();

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

    private void brat( int target, double power){
        motorIntake.setTargetPosition(target);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(power);

    }


    int target_partial = 0;




    private void handleHybridControl(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }


        //CYCLE SOLO
        if(_controller.dpadRightOnce() && cycleSolo == false){
            cycleSolo = true;
        }
        else if(_controller.dpadRightOnce() && cycleSolo == true){
            cycleSolo = false;
        }


        //slow brat
        if(flagSlow == true && runtime.seconds() > 0.5 && runtime.seconds() < 1.0){
            brat(target_partial, 0.25);
        }

        //GUIDE MANUAL
        if(_controller.dpadDownOnce()){
            guide.setPosition((flagGuide)?(1d):(.75d));
            flagGuide = !flagGuide;
        }


        //CYCLE SOLO GROUND
        if (_controller.left_trigger > 0.3 && cycleSolo == true) {
            lifter.setPosition(Lifter.LIFTER_LEVEL.DOWN.ticks);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 100;
            brat(260, 0.25);
            servo180.setPosition(1);
            guide.setPosition(1d);
            flagGuide = false;
            flagUp = false;
            servoExtins.setPosition(1);
            flagExtendo = false;
        }
        //GROUND
        else if (_controller.left_trigger > 0.3) {
            lifter.setPosition(Lifter.LIFTER_LEVEL.DOWN.ticks-100);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 100;
            brat(125, 0.6);
            runtime.reset();
            target_partial=125;
            servo180.setPosition(1);
            guide.setPosition(1d);
            flagGuide = false;
            flagUp = false;
            servoExtins.setPosition(1);
            flagExtendo = false;
            flagSlow = true;
        }
        //LOW
        else if (_controller.leftBumperOnce()) {
            lifter.setPosition(Lifter.LIFTER_LEVEL.DOWN.ticks-100);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 100;
            brat(430, 0.35);
            target_partial = 430;
            servo180.setPosition(1);
            guide.setPosition(1d);
            flagGuide = false;
            flagUp = true;
            flagSlow = false;
        }
        //MID
        else if (_controller.triangleOnce()) {
            lifter.setPosition(760);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 100;
            brat(1450, 0.77);
            runtime.reset();
            target_partial = 1450;
            servo180.setPosition(0);
//            guide.setPosition(.6d);
            flagGuide = true;
            flagUp = true;
            servoExtins.setPosition(1);
            flagExtendo = false;
            flagSlow = true;
        }
        //CYCLE SOLO HIGH
        else if (_controller.right_trigger > 0.3 && cycleSolo == true) {
            lifter.setPosition(1220);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 100;
            brat(1430, 0.6);
            runtime.reset();
            target_partial = 1430;

            servo180.setPosition(0);
            guide.setPosition(.75d);
            flagGuide = true;
            flagUp = true;
            servoExtins.setPosition(1);
            flagExtendo = false;
            flagSlow = true;
        }
        //HIGH
        else if (_controller.right_trigger > 0.3) {
            lifter.setPosition(1400);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 100;
            brat(1400, 0.72);
            runtime.reset();
            target_partial = 1400;

            servo180.setPosition(0);
//            guide.setPosition(.75d);
            flagGuide = true;
            flagUp = true;
            servoExtins.setPosition(1);
            flagExtendo = false;
            flagSlow = true;
        }
        //MANUAL LIFT
        else if(_controller.left_stick_y < -0.3){
            lifter.setPosition(lifter.getPosition()+100);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 0;
        }
        else if(_controller.left_stick_y > 0.3){
            lifter.setPosition(lifter.getPosition()-130);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 0;
        }
        //CON PICAT
        else if (_controller.squareOnce()) {
            lifter.setPosition(400);
            power = 0.6;
            powerOnDec = 0.3;
            decDistance = 0;
            brat(60, 0.25);
            servo180.setPosition(0);
            flagSlow = false;
        }



        //MANUAL BRAT  EXTINS
        if(_controller.right_stick_y < -0.3 && flagExtendo == true){
            brat(motorIntake.getCurrentPosition()+25, 0.15);
            flagSlow = false;
        }
        else if(_controller.right_stick_y > 0.3 && flagExtendo == true){
            brat(motorIntake.getCurrentPosition()-25, 0.15);
            flagSlow = false;
        }

        //MANUAL BRAT DOWN
        else if(_controller.right_stick_y < -0.3 && flagUp == false){
            brat(motorIntake.getCurrentPosition()+36, 0.25);
            flagSlow = false;
        }
        else if(_controller.right_stick_y > 0.3 && flagUp == false){
            brat(motorIntake.getCurrentPosition()-36, 0.25);
            flagSlow = false;
        }

        //MANUAL BTRAT UP
        else if(_controller.right_stick_y < -0.3){
            brat(motorIntake.getCurrentPosition()+55, 0.25); //69
            flagSlow = false;
        }
        else if(_controller.right_stick_y > 0.3){
            brat(motorIntake.getCurrentPosition()-55, 0.25);
            flagSlow = false;
        }

        //TWIST MANUAL
        if(_controller.circleOnce() && flagTwist ==false){
            servo180.setPosition(1);
            flagTwist =  true;
        }
        else if(_controller.circleOnce() && flagTwist == true) {
            servo180.setPosition(0);
            flagTwist = false;
        }

        //EXTENDO
        if(_controller.dpadUpOnce() && flagExtendo == false && cycleSolo == true && flagUp == false){
            servoExtins.setPosition(0.5);
//            sleep(700);
            claw.setPosition(0.48);
            flag =  true;
            flagExtendo =  true;

        }
        else if(_controller.dpadUpOnce() && flagExtendo == true) {
            servoExtins.setPosition(1);
            flagExtendo = false;
        }
        else if(_controller.dpadUpOnce() && flagExtendo == false){
            servoExtins.setPosition(0.5);
            flagExtendo =  true;
        }




        //CLAW
//        if((_controller.rightBumperOnce() && flag ==false) ){
//            claw.setPosition(0.4);
//            flag =  true;
//        }
//        else
        if((_controller.rightBumperOnce() && flag == true && flagUp == true) ) {
            claw.setPosition(0.3);
            flag = false;
        }


    }

    // extend slider slowly with right_stick_x
    // retract slider quickly - min legth

    private void handleDriving(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        if (controller1.rightBumper() ) {

            drive.setWeightedDrivePower(new Pose2d(   //SLOW
                    _controller.left_stick_y * 0.25 +0.00001,
                    _controller.left_stick_x * 0.3,
                    -_controller.right_stick_x * 0.35
            ));
        }
        else{
            drive.setWeightedDrivePower(new Pose2d(     //REGULAR
                    _controller.left_stick_y * 0.7 +0.00001,
                    _controller.left_stick_x * 0.45,
                    -_controller.right_stick_x * 0.35
            ));
        }
        drive.update();


    }

    private void handleDrivingSlowed(ControllerInput _controller) { //FAST
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.setWeightedDrivePower(new Pose2d(
                _controller.left_stick_y * 0.8 +0.00001,
                _controller.left_stick_x * 0.45,
                -_controller.right_stick_x * 0.8
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
        servo180.setDirection(Servo.Direction.REVERSE);
        servo180.scaleRange(.03, 0.7);
        servo180.setPosition(1);

        guide = hardwareMap.get(ServoImplEx.class, "guide");
        guide.setDirection(Servo.Direction.REVERSE);
        guide.setPosition(1);

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setPosition(0.3);

        // slider
        servoExtins = hardwareMap.get(ServoImplEx.class, "servoExtins");
        servoExtins.setPosition(1);


        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake2 = hardwareMap.get(DcMotorEx.class, "motorIntake2");
        motorIntake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake2.setDirection(DcMotorSimple.Direction.REVERSE);


        motorIntake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new LifterNou(hardwareMap, telemetry);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        telemetry.addLine("mechanisms initialized. press START");
        telemetry.update();

//        lifter.setTargetTicks(0, 300);
        // -------------- START --------------
        waitForStart();

//        intake.startIntakeThread();
//        lifter.startLifterThread();
//        intake.endClawProcessesForcibly();

        // intake.servo180.setPosition(1);

        // claw will push too much into the floor otherwise
        // no need to wait until these finish since newer targets take priority in the run() functions
        lifter.setPosition(50);
        power = 0.6;
        powerOnDec = 0.3;
        decDistance = 0;
        brat(30, 0.2);
//        intake.setTargetDeg(0, 15);

        // ------------- DRIVE --------------
        while(opModeIsActive()){
            controller1.update();
            controller2.update();

            telemetry.addData("mode", driveMode);
            telemetry.addData("lateralGrab", lateralGrab);
            telemetry.addData("ticks: ",motorIntake.getCurrentPosition());
            telemetry.addData("CycleSolo", cycleSolo);
            telemetry.update();

            // controller 1
            if (controller1.right_trigger > 0.3 ) {
                handleDrivingSlowed(controller1);

            } else {
                handleDriving(controller1);

            }

            if((controller1.leftBumperOnce() && flag ==false) ){
                claw.setPosition(0.48);
                flag =  true;
            }
            else if((controller1.leftBumperOnce() && flag == true) ) {
                claw.setPosition(0.3);
                flag = false;
            }

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
            if(motorIntake.isBusy()) {
                motorIntake2.setPower(motorIntake.getPower());
            }
            lifter.setPower(power, powerOnDec, decDistance);


        }

        // --------------- STOP ----------------
//        intake.endClawProcessesForcibly();

    }





}
