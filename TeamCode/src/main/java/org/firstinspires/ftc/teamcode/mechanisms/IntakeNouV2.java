package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

public class IntakeNouV2 {

    public ServoImplEx servo180;
    public ServoImplEx claw;
    public ServoImplEx servoExtins;
    public ServoImplEx guide;

    public DcMotorEx leftMotorIntake;
    public DcMotorEx rightMotorIntake;


    public Encoder encoderBrat;


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

        leftMotorIntake = hardwareMap.get(DcMotorEx.class, "leftMotorIntake");
        leftMotorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderBrat = new Encoder(leftMotorIntake);

        rightMotorIntake = hardwareMap.get(DcMotorEx.class, "rightMotorIntake");
        rightMotorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //-------MOTOARE BRAT---------

    public void copyBehaviour(){
        rightMotorIntake.setPower(leftMotorIntake.getPower());
    }

    public void bratSetPosition(int ticks) {
        leftMotorIntake.setTargetPosition(ticks);
        leftMotorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void bratMovement(double power, int decDistance){
        copyBehaviour();

        if(leftMotorIntake.getCurrentPosition() < leftMotorIntake.getTargetPosition()) {
            if(leftMotorIntake.getCurrentPosition() > leftMotorIntake.getTargetPosition() - decDistance){
                leftMotorIntake.setPower(0.25);
            } else {
                leftMotorIntake.setPower(power);
            }
        }

        if(leftMotorIntake.getCurrentPosition() > leftMotorIntake.getTargetPosition()) {
            if(leftMotorIntake.getCurrentPosition() < leftMotorIntake.getTargetPosition() + decDistance){
                leftMotorIntake.setPower(0.25);
            } else {
                leftMotorIntake.setPower(power);
            }

        }
    }

    public void checkHeight(){
        if(leftMotorIntake.getCurrentPosition() >= 1000){
            leftMotorIntake.setPower(0);
            copyBehaviour();
        }
    }

}
