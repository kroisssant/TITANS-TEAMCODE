package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="test intake motors", group="testing")
public class TestIntakeMotors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake2");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.square){
                motorIntake.setPower(.05);
            }

            if(gamepad1.cross){
                motorIntake.setPower(0);
            }
        }
    }
}
