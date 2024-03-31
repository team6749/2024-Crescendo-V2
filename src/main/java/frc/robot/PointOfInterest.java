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

    /**
     * Creates a new PointOfInterest (POI)
     * @param name the name of POI
     * @param translation location on the field in x and y (using blue bottom corner as 0,0) in METERS
     * @param rotation rotation2d of the POI
     * @param toleranceDegrees the "give" of the rotation in degrees. POIs depending on angle should have lower tolerance
     * @param toleranceMeters the "give" of the position. generally should be lower for exact endpoints
     */
    public PointOfInterest(String name, Translation2d translation, Rotation2d rotation, double toleranceDegrees,
            double toleranceMeters) {
        super(translation, rotation);
        this.name = name;
        endPoint = new Pose2d(translation, rotation);
        this.metersTolerance = toleranceMeters;
        this.degreesTolerance = toleranceDegrees;
    }

    /**
     * @param otherPose the pose to compare to the POI pose
     * @return True if otherPose is within metersTolerance of the POI
     */
    public boolean withinMetersTolerance(Pose2d otherPose) {
        Pose2d pose = endPoint.relativeTo(otherPose);
        return pose.getTranslation().getNorm() < metersTolerance;
    }

    /**
     * @param otherPose the pose to compare to the POI pose
     * @return True if otherPose rotation is within degrees tolerance of POI rotation
     */
    public boolean withinDegreesTolerance(Pose2d otherPose) {
        Pose2d pose = endPoint.relativeTo(otherPose);
        return Math.abs(pose.getRotation().getDegrees()) < degreesTolerance;
    }

    /**
     * @param otherPose the pose to compare to the POI pose
     * @return True if withinMetersTolerance AND withinDegreesTolerance are True
     */
    public boolean withinTolerance(Pose2d otherPose) {
        return withinDegreesTolerance(otherPose) && withinMetersTolerance(otherPose);
    }

}
