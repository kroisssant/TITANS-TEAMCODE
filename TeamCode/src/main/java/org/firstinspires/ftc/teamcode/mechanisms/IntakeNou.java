package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.cos;
import static java.lang.Math.abs;

import kotlin.jvm.functions.Function2;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeNou implements Runnable {

    public ServoImplEx servo180;
    public ServoImplEx claw;
    public ServoImplEx servoExtins;
    public DcMotorEx motorIntake;
//    public ThreadScheduler clawThreadScheduler = new ThreadScheduler();

    public Telemetry telemetry;

//    public enum INTAKE_STATE {
//        INIT(0), INSIDE(-170), OUTSIDE(1450);//-150, 1450
//        public int ticks;
//
//        INTAKE_STATE(int ticks) {
//            this.ticks = ticks;
//        }
//    }

    public IntakeNou(@NonNull HardwareMap _hardwareMap) {
        servo180 = _hardwareMap.get(ServoImplEx.class, "servo180");
        servo180.scaleRange(.0, 0.66);
        servo180.setPosition(1);

        claw = _hardwareMap.get(ServoImplEx.class, "claw");
        claw.setPosition(0);

        // slider
        servoExtins = _hardwareMap.get(ServoImplEx.class, "servoExtins");
        servoExtins.setPwmEnable();
        servoExtins.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servoExtins.setDirection(Servo.Direction.FORWARD);
        servoExtins.scaleRange(.4, 1d);
        servoExtins.setPosition(1);

        motorIntake = _hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //----- CLAW -------------

    public boolean isClawOpen = true;

    public void clawClose(){
        claw.setPosition(.35);
        isClawOpen = false;
    }

    public void clawOpen(){
        claw.setPosition(0);
        isClawOpen = true;
    }

    public void clawSetPosition(long _wait, double _target){
        if (_wait <= 0) {
            claw.setPosition(_target);
        } else {
            new Thread(new Thread(() -> {
                try {
                    sleep(_wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                claw.setPosition(_target);
            }));
        }
    }

    public void clawToggle(long _wait){
        if(_wait <= 0){
            if(isClawOpen){
                clawClose();
            } else {
                clawOpen();
            }
        } else {
            new Thread(new Thread(() -> {
                try{
                    sleep(_wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if(isClawOpen){
                    clawClose();
                } else {
                    clawOpen();
                }
            }));
        }
    }

    public void endClawProcessesForcibly(){
//        clawThreadScheduler.killAll();
    }

    //------ SERVO 180 -------

    public void servo180Position(long _wait, double _target){
        if (_wait <= 0) {
            servo180.setPosition(_target);
        } else {
            new Thread(() -> {
                try {
                    sleep(_wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                servo180.setPosition(_target);
            }).start();
        }
    }

    public void rotateClawToAngle(long _wait, int _deg){
        _deg = Range.clip(_deg, 0, 180);

        double target = (double) _deg * 1 / 180;
        servo180Position(_wait, target);
    }

    // ----- SLIDER/SERVO EXTINS ---------

    public double getServoExtPosition(){
        return servoExtins.getPosition();
    }

    public void extendSlider(long _wait, double _target){
        if(_wait <= 0){
            servoExtins.setPosition(_target);
        } else {
            new Thread(() ->  {
                try{
                    sleep(_wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                servoExtins.setPosition((_target < .4)?(.4):(_target)); // due to a retarded java quirk that doesnt let me change _target before using it in a lambda
            }).start();
        }
    }

    // --------- MOTOR INTAKE/ NEW SWING -----

    public int reduceToFirstUnitCircle(int _deg){
        while (_deg < 0){
            _deg = 360 + _deg;
        }
        _deg = _deg % 360;

        return _deg;
    }

    public final double TICKS_TO_DEG = 0.12738;

    public int degFromTicks(double _ticks){
        return (int) ((double) _ticks * TICKS_TO_DEG);
    }

    public int ticksFromDeg(int _deg){
        return (int) (double) (reduceToFirstUnitCircle(_deg) * 1d / TICKS_TO_DEG);
    }

    public boolean isIdle(){return targetTicks == lastTargetTicks;}

    public int getCurrentPosition(){
        return motorIntake.getCurrentPosition();
    }

    public int getCurrentDeg(){
        return reduceToFirstUnitCircle(degFromTicks(getCurrentPosition()));
    }

    public int lastTargetTicks = 0, targetTicks = 0;
    public boolean isReversed = false;
    public boolean kill = false;

    public Thread intakeThread = new Thread(this);

    public boolean manual = false;
    public double manualPower = 0;

    public void toggleManual(){
        if(manual){
            targetTicks = getCurrentPosition();
            lastTargetTicks = targetTicks;
            manualPower = 0;
            manual = false;
        } else {
            manual = true;
            manualPower = 0;
        }
    }

    public void setManualPower(double _pow){
        manualPower = _pow;
    }

    public void startIntakeThread(){
        intakeThread.start();
    }

    public void killIntakeThread(){
        kill = true;
        intakeThread.interrupt();
        motorIntake.setPower(0);
//        lifter.isMotorIntakeMoving = false;
    }

    public void setTargetTicks(long _wait, int _target){
        if(_wait <= 0){
            targetTicks = _target;
        } else {
            new Thread(() -> {
                try{
                    sleep(_wait);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

                targetTicks = _target;
            }).start();
        }
    }

    public void setTargetDeg(long _wait, int _deg){
        setTargetTicks(_wait, ticksFromDeg(_deg));
    }

    public double correction = 0;

    public final double kP = 0.0015, kD = 0.0002, kI = 0, kG = 0.25;

    public final PIDCoefficients pidUp = new PIDCoefficients(0.002, 0.0005, 0.0001);
    public final PIDCoefficients pidDown = new PIDCoefficients(0.002, 0, 0.0002);
    public final PIDCoefficients pidStay = new PIDCoefficients(0.00002, 0, 0);
    public final double kStay = 1E-3;
    public double stayPower = 0;

    public boolean ok = false;
    @Override
    public void run(){

        while (!kill) {

//            if (manual && !kill){
//                internal_setPow(this.manualPower);
//                currentTargetTicks = getCurrentPosition();
//                continue;
//            }

            if (targetTicks != lastTargetTicks && !kill) {
                // New target position came in, go to it
//                lifter.isMotorIntakeMoving = true;
                double currentPosition = getCurrentPosition();

                int lastTargetTicks2 = lastTargetTicks; //to always know how much u wanna go

                lastTargetTicks = targetTicks; // remember this target for later
                if (currentPosition < targetTicks && !kill) {

                    //up
                    PIDFController controller = new PIDFController(pidUp);
                    controller.setTargetPosition(targetTicks);

                    while (targetTicks == lastTargetTicks && !kill && Math.abs(currentPosition - targetTicks) > 30) {
                        currentPosition = getCurrentPosition();
                        double correction = controller.update(currentPosition);
                        double power = Range.clip(correction, -0.0005, 1);
//                        lifter.kIntakeFactor = Range.clip(Math.abs((currentPosition - targetTicks) / (lastTargetTicks2 - targetTicks)), 0, 1);

                        if(motorIntake.getCurrentPosition() <= 0.5 * targetTicks){
                            motorIntake.setPower(power);
                        }
                        else {
                            motorIntake.setPower(power * power * 0.5);
                            ok = true;
                        }
                    }

                } else {
                    ok = false;
                    isReversed = true; //down

                    PIDFController controller = new PIDFController(pidDown);
                    controller.setTargetPosition(targetTicks);

                    while (targetTicks == lastTargetTicks && !kill && Math.abs(currentPosition - targetTicks) > 30) {
                        currentPosition = getCurrentPosition();
                        double correction = controller.update(currentPosition);
                        double power = Range.clip(correction, -1, 0.0005);
                        if(Math.abs(currentPosition - targetTicks) < 100){
//                            lifter.isMotorIntakeMoving = false;
                        }

//                        lifter.kIntakeFactor = Range.clip(Math.abs((currentPosition - targetTicks) / (lastTargetTicks2 - targetTicks)), 0, 1);
                        if(motorIntake.getCurrentPosition() >= 0.7 * lastTargetTicks2){
                            motorIntake.setPower(power);
                        }
                        else {
                            motorIntake.setPower(-2 * power * power * 0.2); // 0.45
                            ok = true;
                        }
                    }
                }
            }

            if(Math.abs(getCurrentPosition() - targetTicks) < 50 && !kill){

                PIDFController controller = new PIDFController(pidStay);
                controller.setTargetPosition(targetTicks);

                //stay
                while(Math.abs(getCurrentPosition() - targetTicks) < 50 && !kill){
                    double currentPosition = getCurrentPosition();
                    double correction = controller.update(currentPosition);
                    int deg = getCurrentDeg();
                    if(deg > 270){
                        continue;
                    }
                    correction += (deg > 100)?(-kStay * deg / 240):(kStay * deg / 100);
                    stayPower = Range.clip(correction, -0.2, 0.15);
                    motorIntake.setPower(stayPower);
                }
            }

        }

        motorIntake.setPower(0);
    }
}
