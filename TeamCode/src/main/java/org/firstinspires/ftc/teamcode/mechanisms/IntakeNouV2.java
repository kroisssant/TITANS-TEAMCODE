package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeNouV2 {

    public ServoImplEx servo180;
    public ServoImplEx claw;
    public ServoImplEx servoExtins;
    public ServoImplEx guide;
    public DcMotorEx motorIntake;

    public IntakeNouV2(HardwareMap hardwareMap, Telemetry telemetry) {
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

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brat(int ticks, double power) {
        motorIntake.setTargetPosition(ticks);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(power);
    }


}
