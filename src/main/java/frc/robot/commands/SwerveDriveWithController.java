// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.SwerveDrivebase;

@SuppressWarnings("unused")
public class SwerveDriveWithController extends Command {
    /** Creates a new SwerveDriveController. */
    private SwerveDrivebase swerveDriveSubsystem;
    private XboxController controller;
    private ChassisSpeeds desiredSpeeds;
    Translation2d finalPose;

    private SlewRateLimiter xVelocitySlew = new SlewRateLimiter(18);
    private SlewRateLimiter yVelocitySlew = new SlewRateLimiter(18);
    private SlewRateLimiter thetaSlew = new SlewRateLimiter(360);

    public SwerveDriveWithController(SwerveDrivebase subsystem, XboxController controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        swerveDriveSubsystem = subsystem;
        this.controller = controller;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // angling + drift prevention, negated to comply with wplib
        double thetaJoystickInput = -controller.getRightX();
        if (Math.abs(thetaJoystickInput) < JoystickConstants.deadZoneRotation) {
            thetaJoystickInput = 0;
        }

        double yJoystickInput = -controller.getLeftY();
        double xJoystickInput = -controller.getLeftX();
        // prevents drifting, if total y joystick + x joystick is less than the dead
        // range, move
        if (magnitude(yJoystickInput, xJoystickInput) < JoystickConstants.deadZoneRange) {
            xJoystickInput = 0;
            yJoystickInput = 0;
        }

        double xSpeedms = joystickResponseCurve(xJoystickInput) * JoystickConstants.maxLinearSpeedms;
        double ySpeedms = joystickResponseCurve(yJoystickInput) * JoystickConstants.maxLinearSpeedms;
        double thetaSpeedRad = joystickResponseCurve(thetaJoystickInput)
                * Math.toRadians(JoystickConstants.maxRotationalSpeedDegrees);

        xSpeedms = xVelocitySlew.calculate(xSpeedms);
        ySpeedms = yVelocitySlew.calculate(ySpeedms);
        thetaSpeedRad = thetaSlew.calculate(thetaSpeedRad);

        switch (swerveDriveSubsystem.getSelectedDriveMode()) {
            case RobotOriented:
                // put robot oriented drive here.
                desiredSpeeds = new ChassisSpeeds(ySpeedms, xSpeedms, thetaSpeedRad);
                break;
            case FieldOriented:
                Rotation2d robotOffsetToAlliance = swerveDriveSubsystem.poseEstimator.getEstimatedPosition()
                        .getRotation();
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeedms, -xSpeedms,
                            thetaSpeedRad, robotOffsetToAlliance);

                } else {
                    desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeedms, xSpeedms,
                            thetaSpeedRad, robotOffsetToAlliance);

                }
                // put field oriented drive here.
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

    /**
     * Sets the joystick gain adjustment to provide a smooth curve for the input.
     * This allows the driver to have more control with the small movements
     * and then will ramp up the inputs as the joystick moves further from the
     * resting position
     * 
     * @param input literal reading from the joystick
     * @return
     */
    public double joystickResponseCurve(double input) {
        return (JoystickConstants.joystickLinearityAdjustment * (Math.pow(input, 5)))
                + ((1 - JoystickConstants.joystickLinearityAdjustment) * input);
    }

    double magnitude(double x, double y) {
        final double xSquared = Math.pow(x, 2);
        final double ySquared = Math.pow(y, 2);

        return Math.sqrt(xSquared + ySquared);
    }

}
