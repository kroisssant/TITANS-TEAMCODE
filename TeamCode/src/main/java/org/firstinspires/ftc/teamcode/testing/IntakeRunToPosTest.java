package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="intake run to position", group="testing")
public class IntakeRunToPosTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorIntakeLeft = hardwareMap.get(DcMotorEx.class, "motorIntake");
        DcMotorEx motorIntakeRight = hardwareMap.get(DcMotorEx.class, "motorIntake2");
        motorIntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorIntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("pos", motorIntakeLeft.getCurrentPosition());
            telemetry.addData("pow", motorIntakeRight.getPower());
            telemetry.update();

            if(gamepad1.square){
                motorIntakeLeft.setTargetPosition(400);
                motorIntakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorIntakeLeft.setPower(.2);
            }

            if(gamepad1.cross || gamepad1.circle){
                motorIntakeLeft.setPower(0);
                motorIntakeRight.setPower(0);
            }

            motorIntakeRight.setPower(motorIntakeLeft.getPower());
        }
    }
}
