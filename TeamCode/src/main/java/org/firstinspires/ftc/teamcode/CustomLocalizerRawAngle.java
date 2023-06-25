package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

public class CustomLocalizerRawAngle implements Localizer {

    public static double ODOMETRY_WEIGHT = 0.5; // must add up to 1
    public static double IMU_WEIGHT = 0.5;

    private StandardTrackingWheelLocalizer wheelLocalizer;
    private HeadingOnlyLocalizer headingLocalizer;

    private Pose2d poseEstimate = new Pose2d(100, 0, 0); // avoid nullptr exception

    private Telemetry telemetry;

    public CustomLocalizerRawAngle(StandardTrackingWheelLocalizer wheelLocalizer, HeadingOnlyLocalizer headingLocalizer, Telemetry telemetry) {
        this.wheelLocalizer = wheelLocalizer;
        this.headingLocalizer = headingLocalizer;
        this.telemetry = telemetry;

        this.wheelLocalizer.setPoseEstimate(poseEstimate);

    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
        wheelLocalizer.setPoseEstimate(pose2d);
    }

    //@Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return wheelLocalizer.getPoseVelocity();
    }

    @Override
    public void update() {
        //wheelLocalizer.setPoseEstimate(poseEstimate);

        wheelLocalizer.update();
        headingLocalizer.update();

        Pose2d odoPose = wheelLocalizer.getPoseEstimate();
        double imuHeading = headingLocalizer.getPoseEstimate().getHeading();

        telemetry.addData("position", odoPose.getX() + "  " + odoPose.getY());
        telemetry.addData("odo angle", Math.toDegrees(odoPose.getHeading()));
        telemetry.addData("imu heading", Math.toDegrees(imuHeading));


        double odoX = odoPose.getX();
        double odoY = odoPose.getY();
        double odoHeading = odoPose.getHeading();

        double fusedHeading = odoHeading * ODOMETRY_WEIGHT + imuHeading * IMU_WEIGHT;

        telemetry.addData("fused heading", Math.toDegrees(fusedHeading));
        telemetry.update();

        this.setPoseEstimate(new Pose2d(odoX, odoY, fusedHeading));
    }
}

