//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//import androidx.annotation.Nullable;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.localization.Localizer;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
//
//public class CustomLocalizer implements Localizer {
//
//    public static double ODOMETRY_WEIGHT = 0.6; // must add up to 1
//    public static double IMU_WEIGHT = 0.4;
//
//    private StandardTrackingWheelLocalizer wheelLocalizer;
//    private HeadingOnlyLocalizer headingLocalizer;
//
//    private Pose2d prevOdoPose = new Pose2d(0, 0, 0);
//    private Pose2d prevImuPose = new Pose2d(0, 0, 0);
//
//    private Pose2d poseEstimate = new Pose2d(0, 0, 0); // avoid nullptr exception
//
//    public CustomLocalizer(StandardTrackingWheelLocalizer wheelLocalizer, HeadingOnlyLocalizer headingLocalizer) {
//        this.wheelLocalizer = wheelLocalizer;
//        this.headingLocalizer = headingLocalizer;
//    }
//
//    @NonNull
//    @Override
//    public Pose2d getPoseEstimate() {
//        return poseEstimate;
//    }
//
//    @Override
//    public void setPoseEstimate(@NonNull Pose2d pose2d) {
//        this.poseEstimate = pose2d;
//        wheelLocalizer.setPoseEstimate(pose2d); // we obviously don't want (0, 0, 0) as initial pose
//        headingLocalizer.setPoseEstimate(pose2d); // correct heading too (also we might want it to be say pi or smth)
//        this.prevOdoPose = pose2d;
//    }
//
//    //@Nullable
//    @Override
//    public Pose2d getPoseVelocity() {
//        return wheelLocalizer.getPoseVelocity(); // why always return null???
//        // return headingLocalizer.getPoseVelocity();
//    }
//
//    @Override
//    public void update() {
//        wheelLocalizer.update();
//        headingLocalizer.update();
//
//        Pose2d odoPose = wheelLocalizer.getPoseEstimate();
//        Pose2d imuPose = headingLocalizer.getPoseEstimate();
//
//        double odoDeltaX = odoPose.getX() - prevOdoPose.getX();
//        double odoDeltaY = odoPose.getY() - prevOdoPose.getY();
//        double odoDeltaHeading = odoPose.getHeading() - prevOdoPose.getHeading();
//        double imuDeltaHeading = preImuPose.getHeading() - imuPose.getHeading();
//
//        double fusedDeltaHeading = odoDeltaHeading * ODOMETRY_WEIGHT + imuDeltaHeading * IMU_WEIGHT;
//
//        this.setPoseEstimate(this.poseEstimate.plus(new Pose2d(odoDeltaX, odoDeltaY, fusedDeltaHeading)));
//
//        this.prevImuPose = imuPose;
//        this.prevOdoPose = odoPose;
//
//        /*
//
//        wheelLocalizer.setPoseEstimate(poseEstimate);
//        wheelLocalizer.update();
//        odoPose = wheelLocalizer.getPoseEstimate();
//
//        Pose2d odoPoseDelta = odoPose.minus(prevOdoPose);
//        double odoHeadingDelta = odoPoseDelta.getHeading();
//
//        headingLocalizer.update();
//        imuHeading = headingLocalizer.getPoseEstimate().getHeading();
//        double imuHeadingDelta = imuHeading - prevHeading;
//
//        double fusedHeadingDelta = ODOMETRY_WEIGHT * odoHeadingDelta + IMU_WEIGHT * imuHeadingDelta;
//        poseEstimate = poseEstimate.plus(new Pose2d(0,0, fusedHeadingDelta));
//
//        prevOdoPose = odoPose;
//        prevHeading = imuHeading;
//        */
//    }
//}
