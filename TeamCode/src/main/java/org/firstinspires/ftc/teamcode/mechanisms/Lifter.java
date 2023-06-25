package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.teleop.Drive;

import kotlin.jvm.functions.Function2;

public class Lifter implements Runnable {

    public DcMotorEx leftLifter;
    protected DcMotorEx rightLifter;
    protected Encoder lifterEncoder;

    public boolean manual = false;

    public double finalLifterTicks;

    public Double getCurrent(CurrentUnit amps) {
        return rightLifter.getCurrent(CurrentUnit.AMPS);
    }

    public static enum LIFTER_LEVEL {
        HIGH(2700), LOW(1050), MID(1750), DOWN(0);

        public int ticks;

        LIFTER_LEVEL(int ticks) {
            this.ticks = ticks;
        }
    }

    public Lifter(@NonNull HardwareMap _hardwareMap, Telemetry _telemetry) {

        leftLifter = _hardwareMap.get(DcMotorEx.class, "lifterLeft");
        rightLifter = _hardwareMap.get(DcMotorEx.class, "lifterRight");

        lifterEncoder = new Encoder(leftLifter);

        leftLifter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLifter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kill = false;
    }

    public void internal_setPow(double _pow) {
        leftLifter.setPower(_pow);
        rightLifter.setPower(_pow);
    }

    public double getCurrentPosition() {
        return leftLifter.getCurrentPosition();
    }

    public void killLifterThread() {
        kill = true;
        currentTargetTicks = 0;
        prevTicks = 0;
        lifterStay = false;
        manual = false;
        manualPower = 0;
    }

    public void setTargetTicks(long _wait, double _target) {
        new Thread(() -> {
            if (_wait != 0) {
                try {
                    Thread.sleep(_wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            currentTargetTicks = _target;
        }).start();
    }

    protected double prevTicks = 0;
    public volatile double currentTargetTicks = 0;
    public volatile boolean kill = false;

    public PIDCoefficients upPIDFCoefficients = new PIDCoefficients(0.0034, 0, 0.00044);
    public PIDCoefficients downPIDFCoefficients = new PIDCoefficients(0.00000009, 0, 0.0000);
    public PIDCoefficients stayPIDFCoefficients = new PIDCoefficients(0.0015, 0, 0);

    public static double kG = 0.001;

    public double maxUpPower = 0.9;

    public double manualPower = 0d;

    public boolean lifterStay = true;

    public void setManualPower(double _power){
        manualPower = _power;
    }

    @Override
    public void run() {

        currentTargetTicks = 0;
        prevTicks = 0;
        lifterStay = true;

        while (!kill) {

            if (manual && !kill){
                internal_setPow(this.manualPower);
                currentTargetTicks = getCurrentPosition();
                continue;
            }

            if (currentTargetTicks != prevTicks && !kill) {
                // New target position came in, go to it
                double currentPosition = getCurrentPosition();
                double initialAbsError = Math.abs(currentPosition - currentTargetTicks); // calculate error

                lifterStay = false;

                prevTicks = currentTargetTicks; // remember this target for later

                if (initialAbsError < 30) {
                    continue; //basically already there
                }

                if (currentPosition < currentTargetTicks && !kill) {
                    // go up
                    PIDFController controllerUp = new PIDFController(upPIDFCoefficients);
                    controllerUp.setTargetPosition(currentTargetTicks);

                    while (currentTargetTicks == prevTicks && !kill && Math.abs(currentPosition - currentTargetTicks) > 70) {
                        currentPosition = getCurrentPosition();
                        double correction = controllerUp.update(currentPosition);
                        double power = Range.clip(correction, 0.1, maxUpPower);
                        internal_setPow(power);
                    }

                    if(Math.abs(currentPosition - currentTargetTicks) <= 70 && !lifterStay){
                        lifterStay = true;
                    }

                } else {
                    // go down
                    PIDFController controllerDown = new PIDFController(downPIDFCoefficients);
                    controllerDown.setTargetPosition(currentTargetTicks);

                    while (currentTargetTicks == prevTicks && !kill && Math.abs(currentPosition - currentTargetTicks) > 70) {
                        currentPosition = getCurrentPosition();
                        double correction = controllerDown.update(currentPosition);

                        double power = Range.clip(correction, -0.2, 0);
                        internal_setPow(power);
                    }
                    if(Math.abs(currentPosition - currentTargetTicks) <= 70 && !lifterStay){
                        lifterStay = true;
                    }
                }
            }

            if(lifterStay && !kill){
                    PIDFController controllerStay = new PIDFController(stayPIDFCoefficients);
                    controllerStay.setTargetPosition(currentTargetTicks);

                    while (currentTargetTicks == prevTicks && !kill && !manual) {

                        double correction = controllerStay.update(getCurrentPosition());
                        double power = Range.clip(correction, 0, 0.4);
                        internal_setPow(power);
                    }
            }

        }
    }
}

