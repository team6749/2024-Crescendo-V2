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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.notemodel.Note;
import frc.robot.subsystems.SwerveDrivebase;

public class SwerveDriveWithController extends Command {
    /** Creates a new SwerveDriveController. */
    private SwerveDrivebase swerveDriveSubsystem;
    private XboxController controller;
    private ChassisSpeeds desiredSpeeds;

	private SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(4);
	private SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(4);
	private SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(8);

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

        double joystickRotation = controller.getRightX();
        if (Math.abs(joystickRotation) < JoystickConstants.deadZoneRotation) {
            joystickRotation = 0;
        }

        double yJoystickInput = limitedJoystickInput(-controller.getLeftY());
        double xJoystickInput = limitedJoystickInput(-controller.getLeftX());
        double thetaJoystickInput = limitedJoystickInput(-joystickRotation);

        if (magnitude(yJoystickInput, xJoystickInput) < JoystickConstants.deadZoneRange) {
            xJoystickInput = 0;
            yJoystickInput = 0;
        }

        double ySpeed = yJoystickInput * JoystickConstants.maxLinearSpeedms;
        double xSpeed = xJoystickInput * JoystickConstants.maxLinearSpeedms;
        double thetaSpeed = thetaJoystickInput * Math.toDegrees(JoystickConstants.maxRotationalSpeedDegrees);

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

        swerveDriveSubsystem.setSubsystemChassisSpeeds(desiredSpeeds);
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

    // limiting joystick sense
    public double limitedJoystickInput(double input) {
        double limitedOutput = (JoystickConstants.limitedOutput * (Math.pow(input, 3))) + ((1 - JoystickConstants.limitedOutput) * input);
        return limitedOutput;
    }

    double magnitude(double x, double y) {
        final double xSquared   = Math.pow(x, 2);
        final double ySquared   = Math.pow(y, 2);
        
        return Math.sqrt(xSquared + ySquared);
      }
    
}
