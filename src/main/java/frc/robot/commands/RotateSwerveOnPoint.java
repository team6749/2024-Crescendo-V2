// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivebase;

public class RotateSwerveOnPoint extends Command {
    /** Creates a new RotateSwerveOnPoint. */
    SwerveDrivebase swerveDrivebase;
    Pose2d finalPose;
    double toleranceDegrees = 7.5;

    public RotateSwerveOnPoint(SwerveDrivebase drivebase) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerveDrivebase = drivebase;
        addRequirements(swerveDrivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finalPose = swerveDrivebase.getPose2d().rotateBy(Rotation2d.fromDegrees(180));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0, 0, getErrorDegrees() < 0 ? Math.PI * 1: -Math.PI * 1));
        // swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0, 0, 1));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    double getErrorDegrees() {
        return swerveDrivebase.getPose2d().relativeTo(finalPose).getRotation().getDegrees();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(getErrorDegrees()) < toleranceDegrees;
    }
}

