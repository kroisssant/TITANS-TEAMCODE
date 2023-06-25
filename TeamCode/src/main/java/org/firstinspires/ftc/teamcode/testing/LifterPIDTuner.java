package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeVechi;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * Ghid de tunare PID Lifter folosind Dashboard
 * Initial kP, kI, kD sunt la 0
 * Se pune kP la o valoare mica tinand cont de ordinul de marime
 * Aici: Eroarea maxima este de 2650 de ticks, deci 1/2650 = 0.00037 ar fi o valoare buna de start (?)
 * deoarece atunci corectia pentru controller (pur P) este de 0.00037 * eroarea = 1 (adica puterea maxima pt motoare)
 * (Obs: Se poate imparti atat targetPosition cat si currentPosition la 100 inainte de a le furniza pid ului pentru a avea
 * coeficienti kP, kI, kD mai mari (ex 0.037) -- nerecomandat momentan)
 * <p>
 * Metoda 1
 * Se mareste kP pana cand lifterul incepe sa oscileze in jurul tintei
 * Se micsoreaza kP putin pana cand oscilatiile sunt minime dar exista
 * Se adauga putin kD pentru amortizarea oscilatiilor
 * <p>
 * <p>
 * Metoda 2
 * Se mareste kP pana cand lifterul se apropie de tinta dar nu o atinge si nu oscileaza
 * Se adauga putin kI ce compenseaza pentru eroarea acumulata in timp pana cand atinge tinta si nu oscileaza
 * sau este agresiv
 **/


@TeleOp
@Config
public class LifterPIDTuner extends LinearOpMode {

    public ControllerInput controller;

    public Lifter lifter;
    public IntakeVechi intakeVechi;
//    public PIDCoefficients upPIDFCoefficients = new PIDCoefficients(0,0,0);
//    public PIDCoefficients downPIDFCoefficients = new PIDCoefficients(0,0,0);

    protected Thread intakeThread, lifterThread;

    public Lifter.LIFTER_LEVEL currentLifterState;
    public IntakeVechi.INTAKE_STATE currentIntakeState;

    public SampleMecanumDrive drive;

    // astea 3 ar tb sa fie vizibile in Dashboard
    public static PIDCoefficients UP_PID = new PIDCoefficients(0.0034, 0, 0.00045);
    public static PIDCoefficients DOWN_PID = new PIDCoefficients(0.000001, 0, 0);
    public static PIDCoefficients STAY_PID = new PIDCoefficients(0.0015, 0, 0);

    public static double maxPower = 0.6;
    public static double kG = 0.001;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        lifter = new Lifter(hardwareMap, telemetry);
        intakeVechi = new IntakeVechi(hardwareMap, true);

        intakeThread = new Thread(intakeVechi);
        lifterThread = new Thread(lifter);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new ControllerInput(gamepad1);


        lifterThread.start();
        intakeThread.start();

        currentIntakeState = IntakeVechi.INTAKE_STATE.INIT;
        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;

        waitForStart();

        intakeVechi.swingSetTargetTicks(0, IntakeVechi.INTAKE_STATE.INSIDE.ticks);

       // new Thread(this::handleTelemetry).start(); //telemetria intr-un thread ca sa fie usor de folosit

        while (opModeIsActive()) {

            controller.update();

            telemetry.addData("Position", lifter.getCurrentPosition());
            telemetry.addData("targetTicks", lifter.currentTargetTicks);
            telemetry.addData("using stay", lifter.lifterStay);
            telemetry.update();

            lifter.stayPIDFCoefficients = STAY_PID;

            if (controller.rightBumperOnce()) {//high
                lifter.maxUpPower = maxPower;
                lifter.upPIDFCoefficients = UP_PID; // la fiecare apasare se actualizeaza coeficientii pid din clasa Lifter
                lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.HIGH.ticks);
                telemetry.addData("targetTicksUP", lifter.currentTargetTicks);
                telemetry.update();
            }

            if (controller.leftBumperOnce()) {//jos
                Lifter.kG = kG;
                lifter.downPIDFCoefficients = DOWN_PID;
                lifter.setTargetTicks(50, Lifter.LIFTER_LEVEL.DOWN.ticks);
            }

            if(controller.circleOnce()){//mid

                lifter.upPIDFCoefficients = UP_PID; // la fiecare apasare se actualizeaza coeficientii pid din clasa Lifter
                lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.MID.ticks);
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

//            drive.LB.setPower(0.4);
//            sleep(3000);
//            drive.LB.setPower(0);
//
//            drive.LF.setPower(0.4);
//            sleep(3000);
//            drive.LF.setPower(0);
//
//            drive.RB.setPower(0.4);
//            sleep(3000);
//            drive.RB.setPower(0);
//
//            drive.RF.setPower(0.4);
//            sleep(3000);
//            drive.RF.setPower(0);

            drive.update();
        }

        lifter.killLifterThread();
        lifter.kill = true;
        intakeVechi.kill = true;
    }

    public void handleTelemetry() {
        telemetry.addData("Position", lifter.getCurrentPosition());
        telemetry.addData("targetTicks", lifter.currentTargetTicks);
        telemetry.update();
        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}