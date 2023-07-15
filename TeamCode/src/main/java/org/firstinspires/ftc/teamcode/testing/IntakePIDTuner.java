package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;

@Config
@TeleOp
@Disabled
public class IntakePIDTuner extends LinearOpMode {
    public ControllerInput controller;
    IntakeVechi intakeVechi;
    protected Thread intakeThread;

    public static double kP = 10.0;
    public static double kI = 0.05;
    public static double kD = 0.0;
    public static double f = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeVechi = new IntakeVechi(hardwareMap, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new ControllerInput(gamepad1);

        PIDFCoefficients coefficients = intakeVechi.getPIDFCoefficients();
        telemetry.addLine("Current PIDF values");
        telemetry.addData("kP", coefficients.p);
        telemetry.addData("kI", coefficients.i);
        telemetry.addData("kD", coefficients.d);
        telemetry.addData("f", coefficients.f);
        telemetry.update();

        waitForStart();
        intakeThread = new Thread(intakeVechi);
        intakeThread.start();

        while (opModeIsActive()) {
            controller.update();

            telemetry.addData("Position", intakeVechi.getCurrentPosition());
            telemetry.update();

            if (controller.rightBumperOnce()) {
                intakeVechi.pidfCoefficients = new PIDFCoefficients(kP, kI, kD, f); // la fiecare apasare se actualizeaza coeficientii
                intakeVechi.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.INSIDE.ticks);
            }

            if (controller.leftBumperOnce()) {
                intakeVechi.pidfCoefficients = new PIDFCoefficients(kP, kI, kD, f); // la fiecare apasare se actualizeaza coeficientii
                intakeVechi.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.OUTSIDE.ticks);
            }
        }
    }
}
