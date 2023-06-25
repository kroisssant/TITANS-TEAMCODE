//package org.firstinspires.ftc.teamcode.testing;
//
//import androidx.annotation.NonNull;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.lib.ControllerInput;
//
//@TeleOp
//public class IntakeTesting extends LinearOpMode {
//    protected DcMotorEx motorIntake;
//    protected ControllerInput controller;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        motorIntake = hardwareMap.get(DcMotorEx.class, "swing");
//        //motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorIntake.setPower(0.0);
//
//        controller = new ControllerInput(gamepad1);
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
////            motorIntake.setPower(-gamepad1.right_stick_y * 0.75);
////            telemetry.addData("Ticks:", motorIntake.getCurrentPosition());
////            telemetry.update();
////        }
//        //init 0     min 50  max 1300
////
////        motorIntake.setPower(0.3);
////        sleep(1000);
////        motorIntake.setPower(-0.3);
////        sleep(1000);
////        motorIntake.setPower(0.0);
//
//        sleep(2000);
//        goToPosition(560);
//        sleep(2000);
//        goToPosition(50);
//        sleep(2000);
//        goToPosition(560);
//        sleep(2000);
//        goToPosition(50);
//
//    }
//
//    void goToPositionThread(long wait, int ticks) {
//        if (wait == 0) {
//            new Thread(() -> {
//                goToPosition(ticks);
//            }).start();
//        }
//        else {
//            new Thread(() -> {
//                try {
//                    Thread.sleep(wait);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//
//                goToPosition(ticks);
//            }).start();
//        }
//    }
//
//    void goToPosition(int ticks) {
//        //motorIntake.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0,0,0,0));
//        motorIntake.setTargetPosition(ticks);
//        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        int direction = ticks > motorIntake.getCurrentPosition() ? 1 : -1;
//
//        motorIntake.setPower(0.4 * direction);
//
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
//            if (timer.seconds() > 2.0) break;
//            telemetry.addData("Ticks:", motorIntake.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.testing;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
public class IntakeTesting extends LinearOpMode {
    protected DcMotorEx motorIntake;
    protected ControllerInput controller;

    public SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new ControllerInput(gamepad1);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drive.RF.setPower(0.4);
            sleep(2000);
            drive.RF.setPower(0);

            drive.RB.setPower(0.4);
            sleep(2000);
            drive.RB.setPower(0);

            drive.LF.setPower(0.4);
            sleep(2000);
            drive.LF.setPower(0);

            drive.LB.setPower(0.4);
            sleep(2000);
            drive.LB.setPower(0);


        }
    }
}

