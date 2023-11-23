package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeV3;

@TeleOp(name = "intake v3 test", group = "testing")
public class IntakeV3Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeV3 intake = new IntakeV3(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("intake arm:", "raw %d | corrected %d | power %f", intake.getRawPosition(), intake.getCurrentPosition(), intake.leftMotorIntake.getPower());
            telemetry.addData("radians", "%f", intake.computeRadians(intake.getCurrentPosition()));
            telemetry.update();
            intake.update();

            if(gamepad1.square)
                intake.armSetPosition(100);

            if(gamepad1.circle)
                intake.armSetPosition(400);

            if(gamepad1.cross)
                intake.armSetPosition(20);
        }
    }
}
