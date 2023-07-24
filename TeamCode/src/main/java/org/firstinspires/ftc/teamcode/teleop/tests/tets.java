package org.firstinspires.ftc.teamcode.teleop.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNouV2;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.mechanisms.LifterNou;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class tets extends LinearOpMode {
    public DcMotorEx leftLifter, rightLifter;
    private Encoder lifterEncoder;

    public DcMotorEx leftMotorIntake;
    public DcMotorEx rightMotorIntake;

    int lifterPos, intakePos;
    private ControllerInput controller1, controller2;
    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);
        leftLifter = hardwareMap.get(DcMotorEx .class, "lifterLeft");
        rightLifter = hardwareMap.get(DcMotorEx.class, "lifterRight");

        lifterEncoder = new Encoder(leftLifter);

        leftMotorIntake = hardwareMap.get(DcMotorEx.class, "leftMotorIntake");
        rightMotorIntake = hardwareMap.get(DcMotorEx.class, "rightMotorIntake");



        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            lifterPos = lifterEncoder.getCurrentPosition();
            intakePos = leftMotorIntake.getCurrentPosition();

            telemetry.addData("Lifter Pos", lifterPos);
            telemetry.addData("Intake Pos", intakePos);
            telemetry.update();
        }
    }
}

