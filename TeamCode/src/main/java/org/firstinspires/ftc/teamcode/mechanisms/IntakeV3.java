package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.util.ArmControllerPID;

public class IntakeV3 {
    public ServoImplEx servo180;
    public ServoImplEx claw;
    public ServoImplEx servoExtender, servoExtinsLongFirst, servoExtinsLongSecond;

    public DcMotorEx leftMotorIntake;
    public DcMotorEx rightMotorIntake;

    public IntakeV3(@NonNull HardwareMap hardwareMap) {
        servo180 = hardwareMap.get(ServoImplEx.class, "servo180");
        servo180.setDirection(Servo.Direction.REVERSE);
        servo180.scaleRange(.03, 0.7);
        servo180.setPosition(1);
        servo180Target = 1;

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setPosition(0.3);
        clawTarget = .3;

        // slider
        servoExtender = hardwareMap.get(ServoImplEx.class, "servoExtins");
        servoExtender.setPosition(1);
        extenderTarget = 1;
        /*servoExtinsLongFirst = hardwareMap.get(ServoImplEx.class, "servoExtinsLongFirst");
        servoExtinsLongSecond = hardwareMap.get(ServoImplEx.class, "servoExtinsLongSecound");*/

        leftMotorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        leftMotorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightMotorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake2");
        rightMotorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidf.setPowerLimits(-0.01, 1);
    }

    // ---------- ARM UTILITIES ------------

    public int getCurrentPosition(){
        return currentPosition;
    }

    public int getRawPosition(){
        return currentMeasurement;
    }

    public static final double ticksToRadians = Math.PI / 2600;
    public static final double measurementGain = 0.2;
    public int lastPosition = 0, currentPosition = 0, currentMeasurement = 0;
    public final ArmControllerPID pidf = new ArmControllerPID(.003, 0, 0, .1);

    public static final double initRadians = -Math.PI / 3;

    public int applyLowPassFilter(){
        return (int) (lastPosition * measurementGain + currentMeasurement * (1 - measurementGain));
    }

    public double computeRadians(int ticks){
        return ticksToRadians * ticks;
    }

    //--------- UPDATE METHOD --------

    public void update(){
        updateArm();
        armLimitCheck();
    }

    // ---------- ARM ----------------

    public void updateArm(){
        lastPosition = currentPosition;
        currentMeasurement = leftMotorIntake.getCurrentPosition();
        currentPosition = applyLowPassFilter();

        double power = pidf.update(currentMeasurement, initRadians + computeRadians(currentPosition));

        if(Math.abs(currentPosition - pidf.getTarget()) < 10){
            power = pidf.kCos * Math.cos(computeRadians(currentPosition));
        }

        leftMotorIntake.setPower(power);
        rightMotorIntake.setPower(power);
    }

    public void armLimitCheck(){
        if(currentMeasurement > 2000 || currentMeasurement < -200){
            leftMotorIntake.setPower(0);
            rightMotorIntake.setPower(0);
            throw new RuntimeException("Intake arm exceeded safety position limits.");
        }
    }

    public void armSetPosition(int target) {
        pidf.setTarget(target);
        pidf.resetSum();
    }

    // ---------- CLAW, EXTENDER, SERVO180 ------------

    public double servo180Target, extenderTarget, clawTarget;

    public void setServo180Target(double target){
        servo180Target = target;
    }

    public void setClawTarget(double target){
        clawTarget = target;
    }

    public void setExtenderTarget(double target){
        extenderTarget = target;
    }

    public void updateServos(){
        inferOpenState();
        claw.setPosition(clawTarget);
        servo180.setPosition(servo180Target);
        servoExtender.setPosition(extenderTarget);
    }

    // ------------ SERVO UTILITIES -----------

    private double clawOpen = .3, clawClosed = .48;
    private boolean isClawOpen = true;

    public void configureClaw(double open, double closed){
        clawOpen = open;
        clawClosed = closed;
    }

    private void inferOpenState(){
        isClawOpen = clawTarget < .4;
    }

    public void openClaw(){
        clawTarget = clawOpen;
        isClawOpen = true;
    }

    public void closeClaw(){
        clawTarget = clawClosed;
        isClawOpen = false;
    }

    public void toggleClaw(){
        clawTarget = (isClawOpen)?(clawClosed):(clawOpen);
        isClawOpen = !isClawOpen;
    }
}
