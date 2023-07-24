package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNouV2;
import org.firstinspires.ftc.teamcode.mechanisms.LifterNou;

@TeleOp
public class testMecanisme extends LinearOpMode {
    IntakeNouV2 intake;
    LifterNou lifter;
    ControllerInput controller1, controller2;
    int decDistance;
    int decDistanceIntake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new IntakeNouV2(hardwareMap, telemetry);
        lifter = new LifterNou(hardwareMap, telemetry);
        lifter.setPosition(50);
        decDistance = 0;

        intake.bratSetPosition(50);
        decDistanceIntake = 0;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            //HIGH//
            if (controller1.triangleOnce()){
                lifter.setPosition(tetsConfig.lifterHigh);
                decDistance = 200;
            }
            //LOW//
            if (controller1.squareOnce()){
                lifter.setPosition(tetsConfig.lifterLow);
                decDistance = 200;

            }
            //MID//
            if (controller1.circleOnce()){
                lifter.setPosition(tetsConfig.lifterMid);
                decDistance = 200;

            }
            if (controller1.leftBumperOnce()){
                intake.bratSetPosition(tetsConfig.intake1);
                decDistanceIntake = 200;
            }
            if (controller1.rightBumperOnce()){
                intake.bratSetPosition(tetsConfig.intake2);
                decDistanceIntake = 200;
            }
            lifter.setPower(0.3, 0.1, decDistance);
            intake.setPower(0.3, decDistanceIntake);

            intake.servoExtins.setPosition(tetsConfig.servo0);
            intake.extendoElLongo(tetsConfig.servo1, tetsConfig.servo2);
        }


    }
}

