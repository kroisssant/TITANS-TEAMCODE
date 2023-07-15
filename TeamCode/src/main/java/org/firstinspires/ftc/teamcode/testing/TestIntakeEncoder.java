package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;

@TeleOp
@Disabled
public class TestIntakeEncoder extends LinearOpMode {
    IntakeVechi intakeVechi;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeVechi = new IntakeVechi(hardwareMap, true);
        intakeVechi.motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("current: ", intakeVechi.motorIntake.getCurrentPosition());
            telemetry.update();

        }
    }
}
