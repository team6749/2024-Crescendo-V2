// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWithController extends Command {
	/** Creates a new SwerveDriveController. */
	private SwerveSubsystem swerveDriveSubsystem;
	private XboxController controller;
	private ChassisSpeeds desiredSpeeds;

	private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
	private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
	private SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(5);

	public SwerveDriveController(SwerveSubsystem subsystem, XboxController controller) {
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

		double xSpeed = xSpeedLimiter.calculate(-controller.getLeftX());
		double ySpeed = ySpeedLimiter.calculate(-controller.getLeftY());
		double thetaSpeed = thetaSpeedLimiter.calculate(-controller.getRightX()); // angular speed

		switch (swerveDriveSubsystem.getSelectedDriveMode()) {
			case RobotOriented:
				// put robot oriented drive here.
				desiredSpeeds = new ChassisSpeeds(ySpeed, xSpeed, thetaSpeed);
				break;
			case FieldOriented:
				// put field oriented drive here.
				desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
						thetaSpeed, swerveDriveSubsystem.getGyroRotation());
				break;
		}

		if (Math.abs(ySpeed) < 0.15 && Math.abs(xSpeed) < 0.15
				&& Math.abs(thetaSpeed) < 0.15) {
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
}
