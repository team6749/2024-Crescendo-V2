// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.DriveOrientation;

public class SwerveDrivebase extends SubsystemBase {
    /** Creates a new SwerveDrivebase. */
    public SwerveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveDriveOdometry odometry;
    public SwerveModuleState[] states;
    public SwerveDrivePoseEstimator poseEstimator;
    public SendableChooser<DriveOrientation> orientation = new SendableChooser<DriveOrientation>();

    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public static final Field2d field = new Field2d();

    public SwerveDrivebase(SwerveModule[] modules) {
        this.modules = modules;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
