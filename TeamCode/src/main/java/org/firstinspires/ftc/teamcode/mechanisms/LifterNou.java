package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

public class LifterNou {
    public static enum LIFTER_LEVEL {
        HIGH(2700), LOW(1050), MID(1750), DOWN(0);

        public int ticks;

        LIFTER_LEVEL(int ticks) {
            this.ticks = ticks;
        }
    }

    public DcMotorEx leftLifter, rightLifter;
    private Encoder lifterEncoder;

    public LifterNou (HardwareMap hardwareMap, Telemetry telemetry){
        leftLifter = hardwareMap.get(DcMotorEx.class, "lifterLeft");
        rightLifter = hardwareMap.get(DcMotorEx.class, "lifterRight");

        lifterEncoder = new Encoder(leftLifter);

        leftLifter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLifter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPosition(int ticks) {
        leftLifter.setTargetPosition(ticks);

        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void copyBehaviour() {
        if(leftLifter.isBusy()) {
            rightLifter.setPower(leftLifter.getPower());
        }
    }

    public void setPower(boolean manual, double power, double powerOnDec) {
        if(!manual) {
            if(lifterEncoder.getCurrentPosition() > leftLifter.getTargetPosition() - 100) {
                leftLifter.setPower(powerOnDec);
            } else if(lifterEncoder.getCurrentPosition() > leftLifter.getTargetPosition()) {
                leftLifter.setPower(power);
            }
            if(lifterEncoder.getCurrentPosition() < leftLifter.getTargetPosition() + 100) {
                leftLifter.setPower(powerOnDec);
            } else if(lifterEncoder.getCurrentPosition() < leftLifter.getTargetPosition()) {
                leftLifter.setPower(power);
            }
        }
    }
}
