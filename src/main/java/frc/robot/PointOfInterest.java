// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PointOfInterest extends Pose2d {

    private Pose2d endPoint;
    double metersTolerance;
    double degreesTolerance;
    public String name;

    public PointOfInterest(String name, Translation2d translation, Rotation2d rotation, double toleranceDegrees,
            double toleranceMeters) {
        super(translation, rotation);
        this.name = name;
        endPoint = new Pose2d(translation, rotation);
        this.metersTolerance = toleranceMeters;
        this.degreesTolerance = toleranceDegrees;
    }

    public boolean withinMetersTolerance(Pose2d otherPose) {
        Pose2d pose = endPoint.relativeTo(otherPose);
        return pose.getTranslation().getNorm() < metersTolerance;
    }

    public boolean withinDegreesTolerance(Pose2d otherPose) {
        Pose2d pose = endPoint.relativeTo(otherPose);
        return Math.abs(pose.getRotation().getDegrees()) < degreesTolerance;
    }

    public boolean withinTolerance(Pose2d otherPose) {
        return withinDegreesTolerance(otherPose) && withinMetersTolerance(otherPose);
    }

}
