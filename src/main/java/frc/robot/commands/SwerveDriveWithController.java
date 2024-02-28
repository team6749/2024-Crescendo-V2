// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivebase;

public class SwerveDriveWithController extends Command {
    /** Creates a new SwerveDriveController. */
    private SwerveDrivebase swerveDriveSubsystem;
    private XboxController controller;
    private ChassisSpeeds desiredSpeeds;

	private SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(4);
	private SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(4);
	private SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(8);

    private PIDController aimbot = new PIDController(1, 0, 0);

    public SwerveDriveWithController(SwerveDrivebase subsystem, XboxController controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        swerveDriveSubsystem = subsystem;
        this.controller = controller;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

		double xSpeed = xAccelerationLimiter.calculate(-controller.getLeftX() * 3);
		double ySpeed = yAccelerationLimiter.calculate(-controller.getLeftY() * 3);
		double thetaSpeed = thetaSpeedLimiter.calculate(-controller.getRightX() * 3.5); // angular speed

        switch (swerveDriveSubsystem.getSelectedDriveMode()) {
            case RobotOriented:
                // put robot oriented drive here.
                desiredSpeeds = new ChassisSpeeds(ySpeed, xSpeed, thetaSpeed);
                break;
            case FieldOriented:
                // put field oriented drive here.
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                        thetaSpeed, swerveDriveSubsystem.getRotation2d());
                break;
        }
        
        if(controller.getXButton()) {
            desiredSpeeds = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond + 0.5, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond + aimbot.calculate(/* target note angle from center */, 0));
        }

        if (Math.abs(ySpeed) < 0.25 && Math.abs(xSpeed) < 0.25
                && Math.abs(thetaSpeed) < 0.2) {
            swerveDriveSubsystem.setSubsystemModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            });
        } else {
            swerveDriveSubsystem.setSubsystemChassisSpeeds(desiredSpeeds);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
