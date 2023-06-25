package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class HeadingOnlyLocalizer implements Localizer {

    private BNO055IMU imu;
    Pose2d poseEstimate = new Pose2d(0, 0, 0);
    double initialHeadingRadians = 0;

    public HeadingOnlyLocalizer(BNO055IMU imu, double initialHeadingRadians) {
        this.imu = imu;
        this.initialHeadingRadians = initialHeadingRadians;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d; // correct angle; X and Y are irrelevant
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        poseEstimate = new Pose2d(0, 0, Angle.norm(getRawExternalHeading()) + initialHeadingRadians); // add initial heading
    }
}
