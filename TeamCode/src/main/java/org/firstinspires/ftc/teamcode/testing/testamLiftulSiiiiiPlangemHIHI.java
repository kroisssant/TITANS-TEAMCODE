package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNouV2;
import org.firstinspires.ftc.teamcode.mechanisms.LifterNou;

@TeleOp(group = "test", name = "Mama mea e florareasa")
public class testamLiftulSiiiiiPlangemHIHI extends LinearOpMode {

    private LifterNou lifter;
    private IntakeNouV2 intake;
    private ControllerInput controller1;

    private int decDistance = 450;
    private int decDisIntake = 100;
    double power1, power11;
    double power2, power21;

    @Override
    public void runOpMode() throws InterruptedException {
        lifter = new LifterNou(hardwareMap, telemetry);
        intake = new IntakeNouV2(hardwareMap, telemetry);
        controller1 = new ControllerInput(this.gamepad1);

        lifter.setPosition(50);
        intake.bratSetPosition(50);

        waitForStart();

        while(opModeIsActive()){
            controller1.update();

//            intake.bratMovement(0.4, decDistance);
            lifter.setPower(0.7, 0.3, decDistance);
            lifter.copyBehaviour();
            intake.setPower(0.01, decDisIntake);

            //GROUND
            if(controller1.left_trigger > 0.3){
                lifter.setPosition(LifterNou.LIFTER_LEVEL.DOWN.ticks);
                decDistance = 450;
            }
            //LOW
            else if(controller1.leftBumperOnce())  {
                lifter.setPosition(LifterNou.LIFTER_LEVEL.LOW.ticks);
                decDistance = 450;
            }
            //MID
            else if(controller1.triangleOnce()) {
                lifter.setPosition(LifterNou.LIFTER_LEVEL.MID.ticks);
                decDistance = 450;
            }
            //HIGH
            else if(controller1.rightBumperOnce()) {
                lifter.setPosition(LifterNou.LIFTER_LEVEL.HIGH.ticks);
                decDistance = 450;
            }
            // manual
            if(controller1.left_stick_y > 0.3 || controller1.left_stick_y < -0.3) {
                lifter.setPosition((int)(lifter.getPosition() + controller1.left_stick_y * 100));
                decDistance = 0;
            }

            if(controller1.dpadUpOnce()) {
                intake.bratSetPosition(420);
                decDisIntake = 100;
            }
            if(controller1.dpadDownOnce()) {
                intake.bratSetPosition(0);
                decDisIntake = 100;
            }

            if(controller1.right_stick_y > 0.3 || controller1.right_stick_y < -.3) {
                intake.bratSetPosition((int)(intake.getPosition() + controller1.right_stick_y * 100));
                decDisIntake = 0;
            }
            power1 = intake.leftMotorIntake.getPower();
            power11 = intake.rightMotorIntake.getPower();
            power2 = lifter.leftLifter.getPower();
            power21 = lifter.rightLifter.getPower();

            telemetry.addData("power brat L", power1);
            telemetry.addData("power brat R", power11);
            telemetry.addData("power lifter L", power2);
            telemetry.addData("power lifter R", power21);
            telemetry.addData("target pos lifter", lifter.leftLifter.getTargetPosition());
            telemetry.addData("current pos lifter", lifter.leftLifter.getCurrentPosition());

            telemetry.update();
        }
    }
}
