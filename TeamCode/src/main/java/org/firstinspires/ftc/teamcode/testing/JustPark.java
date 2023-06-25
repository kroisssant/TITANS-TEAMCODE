package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.drive.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.*;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


//------JUST PARK USING TIMER + DETECTION----------------
@Disabled
@Autonomous(name ="autoPark1", group = "_1auto")
public class JustPark extends LinearOpMode {
	public SampleMecanumDrive drive;
	public ElapsedTime timer;
	private OpenCvWebcam webcam;
	private SignalPipeline pipeline = new SignalPipeline(2, new int[] {0, 1, 2});
	private int actualTarget = -1;
	private boolean cameraOK = true;
	private boolean checkParking = false;

	private void _init() {
		_initDetection();
		timer = new ElapsedTime();
		drive = new SampleMecanumDrive(hardwareMap);
	}

	private void _initDetection() {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

		webcam.setMillisecondsPermissionTimeout(3000);
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
			@Override
			public void onOpened(){
				webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
				try{
					Thread.sleep(1000); // always wait before pressing start
				} catch (InterruptedException e){
					;;
				}

				webcam.setPipeline(pipeline);
			}

			@Override
			public void onError(int errorCode){
				telemetry.addLine(String.format("canera failed to open: errcode %d", errorCode));
				telemetry.update();
				cameraOK = false;
			}
		});
	}

	// Thread.sleep() could be the cause of "stuck in stop"
	// so this thread here will be returned by the function and inside
	// runOpMode() checks are done to interrupt the thread when stop is requested
	public Thread park (int target) {
		Thread parkThread = new Thread(() -> {
			if (target < 0 && !checkParking){

				// detection failed so parks in 2
				drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);

				telemetry.addLine("parkingspot 2 automat nedetectat");
				telemetry.update();

				while(timer.milliseconds() < 1750){
					//wait for time to pass
				}
				drive.setMotorPowers(0,0,0,0);
				checkParking = true;

			} else if (target == 0 && !checkParking ){
				//parks in 1 (target 0 == sleeve 1)
				drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);

				telemetry.addLine("parkingspot 1");
				telemetry.update();

				while(timer.milliseconds() < 1500){
					//wait for time to pass
				}

				drive.setMotorPowers(-0.4, 0.4, -0.4, 0.4);

				timer.reset();
				while(timer.milliseconds() < 1900){
					//wait for time to pass
				}
				drive.setMotorPowers(0,0,0,0);
				checkParking = true;

			} else if (target == 1 && !checkParking ){

				//parks in 2 (target 1 == sleeve 2)
				drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);

				telemetry.addLine("parkingspot 2");
				telemetry.update();

				while(timer.milliseconds() < 1750){
					//wait for time to pass
				}
				drive.setMotorPowers(0,0,0,0);
				checkParking = true;

			} else if (target == 2 && !checkParking ){
				//parks in 3 (target 2 == sleeve 3)
				drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);

				telemetry.addLine("parkingspot 3");
				telemetry.update();

				while(timer.milliseconds() < 1500){
					//wait for time to pass
				}

				drive.setMotorPowers(0.4, -0.4, 0.4, -0.4);

				timer.reset();
				while(timer.milliseconds() < 1900){
					//wait for time to pass
				}
				drive.setMotorPowers(0,0,0,0);
				checkParking = true;
			}
		});

		parkThread.start();
		return parkThread;
	}

	@Override
	public void runOpMode(){
		_init();

		waitForStart();
		timer.reset();

		boolean alrkilled = false; // used to make sure all camera threads are killed at stop

		while(opModeIsActive()){
			if (!cameraOK){alrkilled = true; break;}
            if (timer.milliseconds() < 3000){continue;}
			actualTarget = pipeline.targetFound;

			telemetry.addLine(String.format("target: %d", actualTarget));
			telemetry.update();

            pipeline.stopAprilTagDetection();
            alrkilled = true;
		}

		// opmode got stopped before killing the pipeline and camera thread
		if (!alrkilled){
			webcam.stopStreaming();
			webcam.closeCameraDevice();
			pipeline.stopAprilTagDetection();
		}

		// perform the parking
		Thread parkThread = null;
		if (opModeIsActive()){
			parkThread = park(actualTarget);
		}

		while (true){
			if (!opModeIsActive()){
				// if opmode gets stopped kill the parkThread immediately and break out of the loop
				if (parkThread != null){
					parkThread.interrupt();
					drive.setMotorPowers(0,0,0,0);
				}
				break;
			}
		}
	}
}
