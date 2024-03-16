// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class PointOfInterest extends Pose2d {

    Pose2d endPoint;
    double metersTolerance;
    double degreesTolerance;

    public PointOfInterest(String name, Pose2d point, double toleranceDegrees, double toleranceMeters){
        endPoint = point;
        this.metersTolerance = toleranceMeters;
        this.degreesTolerance = toleranceDegrees;
    }

    public Boolean withinMetersTolerance(Pose2d currentPose){
        Pose2d pose = endPoint.relativeTo(currentPose);
        return pose.getTranslation().getNorm() < metersTolerance;
    }

    public boolean withinDegreesTolerance(Pose2d currentPose){
        Pose2d pose = endPoint.relativeTo(currentPose);
        return Math.abs(pose.getRotation().getDegrees()) < degreesTolerance;
    }

}
