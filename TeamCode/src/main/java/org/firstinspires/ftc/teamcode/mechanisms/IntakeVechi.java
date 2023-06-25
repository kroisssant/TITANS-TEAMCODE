package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Thread.sleep;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeVechi implements Runnable {
    protected CRServo leftCRServoIntake;
    protected CRServo rightCRServoIntake;

    public double finalIntakeTicks;
    public Telemetry telemetry;

    public DcMotorEx motorIntake;
    public PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10,0.05,0,0);  // nu sunt utilizati
    /* DEFAULT PIDF COEFFICIENTS in caz ca ceva nu e ok
        p = 10  i = 0.05   d = 0.0   f = 0.0
     */

    protected ServoImplEx paCon; // 1 - inside; 0.2 - outside

    public boolean manual = false;

    public enum INTAKE_STATE {
        INIT(0), INSIDE(-170), OUTSIDE(1450);//-150, 1450
        public int ticks;

        INTAKE_STATE(int ticks) {
            this.ticks = ticks;
        }
    }

    public IntakeVechi(@NonNull HardwareMap _hardwareMap, boolean encoder) {
        leftCRServoIntake = _hardwareMap.get(CRServo.class, "servoL");
        rightCRServoIntake = _hardwareMap.get(CRServo.class, "servoR");

        motorIntake = _hardwareMap.get(DcMotorEx.class, "swing");

        paCon = _hardwareMap.get(ServoImplEx.class, "paCon");
        paCon.setPwmRange(new PwmControl.PwmRange(500, 2500));

        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftCRServoIntake.setDirection(CRServo.Direction.REVERSE);
        rightCRServoIntake.setDirection(CRServo.Direction.FORWARD);

        prevTicks = 0;
        currentTargetTicks = 0;
        isSpinning = false;

        if (!encoder) {
            motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        paConInside(0);
    }

    public void paConInterpolateThread(long wait, double initPose, double finalPose, long ms, int samples) {
        if (wait == 0) {
            paConInterpolate(initPose, finalPose, ms, samples);
            return;
        }
        new Thread(() -> {
            try {
                Thread.sleep(wait);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            paConInterpolate(initPose, finalPose, ms, samples);
        }).start();
    }

    public void setPowSwing(double pow){motorIntake.setPower(pow);}

    private void paConInterpolate(double initPose, double finalPose, long ms, int samples) {
        new Thread(() -> {
            double timeBetweenSamples = ms / (samples * 1.0);
            double positionIncrementPerSample = (finalPose - initPose) / samples;
            double currentPos = initPose;
            for (int i = 0; i < samples; i++) {
                paCon.setPosition(currentPos);
                currentPos += positionIncrementPerSample;

                try {
                    Thread.sleep((long) timeBetweenSamples);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            paCon.setPosition(finalPose); //ensure it's actually there
        }).start();
    }

    public void paConInside(long waitMs) {
        paConGoToPositionThread(0.96, waitMs);
    }

    public void paConGoToPositionThread(double pos, long waitMs) {
        if (waitMs == 0) {
            paCon.setPosition(pos);
            return;
        }
        new Thread(() -> {
            try {
                Thread.sleep(waitMs);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            paCon.setPosition(pos);
        }).start();
    }

    public void paConOutside(long waitMs) {
        paConGoToPositionThread(0.45, waitMs);
    }


    //-------------SWING--------------

    public void swingSetTargetTicks(long _wait, int _target) {
        if (_wait == 0) {
            swingGoToPosition(_target);
        } else {
            new Thread(() -> {
                try {
                    sleep(_wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                swingGoToPosition(_target);
            }).start();
        }
    }

    void swingGoToPosition(int _ticks) {
        currentTargetTicks = _ticks;
    }

    public void killIntakeThread() {
        kill = true;
        currentTargetTicks = 0;
        prevTicks = 0;
    }

    public volatile boolean kill = false;
    public volatile int currentTargetTicks = 0;
    protected int prevTicks = 0;
//    private long lastMillis = 0;

    @Override
    public void run() {

        currentTargetTicks = 0;
        prevTicks = 0;

        while (!kill) {

            if (currentTargetTicks != prevTicks && !kill) {
                motorIntake.setTargetPosition(currentTargetTicks);
                motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                int direction = (currentTargetTicks > prevTicks) ? (1) : (-1);

                motorIntake.setPower(0.8 * direction);
                prevTicks = currentTargetTicks;
                while (/*!Thread.currentThread().isInterrupted() &&*/ !kill) {
                    // telemetry.addData("current: ", getCurrentPosition());
                    // telemetry.addData("target: ", currentTargetTicks);
                    //telemetry.addData("runmode: ", motorIntake.getMode());
                    // telemetry.update();
                    if (currentTargetTicks != prevTicks) break;
                }
                continue; // do this to skip the next line where prevTicks becomes equal to currentTargetTicks
            }
            prevTicks = currentTargetTicks;
        }
    }


    //--------INTAKE SERVOS------------
    public void setCRServosPow(double _power) {
        leftCRServoIntake.setPower(_power);
        rightCRServoIntake.setPower(_power);

    }

    public volatile boolean isSpinning = false;


    public void spinFor(long waitBefore, long spinFor_ms, double power) {
        //if (isSpinning) return;
        new Thread(() -> {
            //if (Thread.currentThread().isInterrupted()) return; //paranoia

            if (waitBefore != 0) {
                try {
                    Thread.sleep(waitBefore);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            isSpinning = true;
            setCRServosPow(power);
            try {
                Thread.sleep(spinFor_ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setCRServosPow(0.0);

            isSpinning = false;
        }).start();
    }

    public PIDFCoefficients getPIDFCoefficients() {
        return motorIntake.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int getCurrentPosition() {
        return motorIntake.getCurrentPosition();
    }
}
