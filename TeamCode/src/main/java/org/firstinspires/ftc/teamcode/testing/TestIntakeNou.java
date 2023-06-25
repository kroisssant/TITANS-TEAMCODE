package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNou;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "intake nou testing")
public class TestIntakeNou extends LinearOpMode {
    public DcMotorEx intakeMtr;
    public IntakeNou intake;
    public ControllerInput controller2;
    public Lifter lifter;
    public Thread lifterThread;
    public Thread intakeThread;

    @Override
    public void runOpMode(){
        lifter = new Lifter(hardwareMap, telemetry);
        controller2 = new ControllerInput(gamepad2);

        intakeThread = new Thread(intake);
        lifterThread = new Thread(lifter);

        waitForStart();

//        intake.startIntakeThread();
        lifterThread.start();
        intakeThread.start();

        int pos = intake.getCurrentPosition();
        int deg = intake.degFromTicks(pos);

        while(opModeIsActive()){
            pos = intake.getCurrentPosition();
            controller2.update();

            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.MID.ticks);

            if(controller2.crossOnce()){
                intake.clawToggle(500);
            }

//            if(controller2.squareOnce()){
//                intake.extendSlider(0, 0);
//            }
//
//            if(controller2.leftBumper()){
//                intake.extendSlider(0, 1);
//            }

            if(controller2.triangleOnce()){
                intake.rotateClawToAngle(0, 45);
            }

            if(controller2.rightBumper()){
                intake.rotateClawToAngle(0, 0);
            }

            if(controller2.circleOnce()){
                intake.extendSlider(0, 0);
            }

            if(controller2.dpadLeft()){
                intake.extendSlider(0, 1);
            }

            if(controller2.dpadDownOnce()){
                intake.setTargetDeg(0, 145 + 55);
//                intake.setTargetTicks(0, 300);

            }

            if(controller2.dpadUpOnce()){
                intake.setTargetDeg(0, 20);
//                intake.setTargetTicks(0, 300);

            }



//            telemetry.addData("delta theta: ", intake.getCurrentDeg() - deg);
//            telemetry.addData("goes against gravity: ", intake.goesAgainstGravity(intake.getCurrentDeg()));
//            telemetry.addData("correction: ", intake.correction);

            telemetry.addData("targetTicks: ", intake.targetTicks);
//            telemetry.addData("currentDegrees: ", intake.getCurrentDeg());
            telemetry.addData("currentTicks: ", intake.getCurrentPosition());
            telemetry.addData("ok ", intake.ok);




            telemetry.update();
        }

        intake.killIntakeThread();
        intakeThread.interrupt();
        intake.kill = true;

        lifterThread.interrupt();
        lifter.killLifterThread();
        lifter.kill = true;
    }
}
