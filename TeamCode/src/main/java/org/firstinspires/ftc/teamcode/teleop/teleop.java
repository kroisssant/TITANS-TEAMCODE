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

    double power, powerOnDec;

    boolean cycleSolo = false;
    boolean manual = false;
    @Override
    public void runOpMode() throws InterruptedException {
        lifter = new LifterNou(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeNouV2(hardwareMap, telemetry);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            controller1.update();
            controller2.update();


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
            drive.update();

            lifter.copyBehaviour();
            lifter.setPower(manual, power, powerOnDec);
        }
    }

    public void mechanism() {

        if(gamepad2.dpadRightOnce() && cycleSolo == false){
            cycleSolo = true;
        }
        else if(gamepad2.dpadRightOnce() && cycleSolo == true){
            cycleSolo = false;
        }
    }
}
