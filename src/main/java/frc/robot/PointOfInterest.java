// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class PointOfInterest {
    Pose2d endPoint;
    double metersTolerance;
    double degreesTolerance;

public PointOfInterest(String name, Pose2d point, double toleranceDegrees, double toleranceMeters){
    endPoint = point;
    this.metersTolerance = toleranceMeters;
    this.degreesTolerance = toleranceDegrees;
    var alliance = DriverStation.getAlliance();

    if(alliance.get() == DriverStation.Alliance.Red){
        endPoint = Constants.SwerveConstants.sideifyPose2d(endPoint);
    }

}
public Twist2d logPoint(Pose2d currentPose){
    return currentPose.log(endPoint);
}

public Boolean withinMetersTolerance(Pose2d currentPose){
    if((currentPose.getX() - endPoint.getX() < metersTolerance) && (currentPose.getY() - endPoint.getY() < metersTolerance)){
        return true;
    }else{
        return false;
    }
}
public boolean withinDegreesTolerance(Pose2d currentPose){
    if(currentPose.getRotation().getDegrees() - endPoint.getRotation().getDegrees() < degreesTolerance){
        return true;
    }else{
        return false;
    }
}


}
