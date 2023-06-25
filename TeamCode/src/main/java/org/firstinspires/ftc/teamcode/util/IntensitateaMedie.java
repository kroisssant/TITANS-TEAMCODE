package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.ArrayList;

@TeleOp
public class IntensitateaMedie extends LinearOpMode {
    public SampleMecanumDrive drive;
    public Lifter lifter;
    public DcMotorEx motor;
    ArrayList<Double> currentMeasurements;


    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "lifterLeft");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime elapsedTime = new ElapsedTime();
        currentMeasurements = new ArrayList<>();

        waitForStart();

        elapsedTime.reset();

        motor.setPower(0.9);
        while (opModeIsActive() && elapsedTime.milliseconds() < 1080) {
            currentMeasurements.add(motor.getCurrent(CurrentUnit.AMPS));
        }
        motor.setPower(0);


        // Calcul medie
        double s = 0;
        for (Double d : currentMeasurements) {
            s += d;
        }

        while(opModeIsActive()) {
            telemetry.addData("Intensitatea medie: ", s / currentMeasurements.size());
            telemetry.update();
        }
    }

}
