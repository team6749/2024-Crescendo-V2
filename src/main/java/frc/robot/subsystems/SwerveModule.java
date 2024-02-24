// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule implements Sendable {
    /** Creates a new SwerveModule. */
    public String name;
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder encoder;
    public Translation2d location;
    public final PIDController velocityPIDController = new PIDController(0.0, 0, 0);
    public final PIDController anglePIDController = new PIDController(0.1, 0, 0);
    public double maxSpeed;



    public SwerveModule(String name, int driveMotorPort, int encoderPort, int angleMotorPort,
            Translation2d locationFromCenter) {
        this.name = name;
        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new TalonFX(angleMotorPort);
        encoder = new CANcoder(encoderPort);
        location = locationFromCenter;
        velocityPIDController.enableContinuousInput(0, 360);
        this.setModuleNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addStringProperty(name, this::getName, this::setName);
        builder.addDoubleProperty("velocity", this::getDriveEncoderVelocity, null);
        builder.addDoubleProperty("position", this::getDriveEncoderPosition, null);
        builder.addDoubleProperty("rotation", this::getRotationEncoder, null);
        builder.addDoubleProperty("Max Speed", this::getMaxSpeed, this::setMaxSpeed);

        SmartDashboard.putData("swerve " + name + " velocity pid", velocityPIDController);
        SmartDashboard.putData("swerve " + name + " angle pid", anglePIDController);
        builder.setSafeState(this::stop);
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    /**
     * @return the velocity of robot in meters per second
     */
    public double getDriveEncoderVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() / (Constants.SwerveConstants.swerveGearRatio)
                * (Math.PI * Constants.SwerveConstants.swerveWheelDiameterMeters);
    }

    /**
     * @return the position of the drive motor
     */
    public double getDriveEncoderPosition() {
        return driveMotor.getPosition().getValueAsDouble() / (Constants.SwerveConstants.swerveGearRatio)
                * (Math.PI * Constants.SwerveConstants.swerveWheelDiameterMeters);

    }

    /**
     * @return the rotation of a module in degrees
     */
    public double getRotationEncoder() {
        return encoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    /**
     * @return the SwerveModuleState of the module - contains the velocity and
     *         rotation of the module
     *         see
     *         https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     *         for more info
     */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveEncoderVelocity(), Rotation2d.fromDegrees(getRotationEncoder()));
    }

    /**
     * @return the position of the module initally
     *         see
     *         https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
     *         for more info
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getRotationEncoder()));

    }

    /**
     * sets the swerve module to the desiredState
     * 
     * @param desiredState the new desired state of a SwerveModule
     */
    public void setSwerveModuleState(SwerveModuleState desiredState) {
        SmartDashboard.putNumber(name + " Desired Chassis States", desiredState.speedMetersPerSecond);
        // prevents wheels from resetting back to straight orientation
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                Rotation2d.fromDegrees(getRotationEncoder()));

        // // Calculate the drive output from the drive PID controller.
        final double driveOutput = velocityPIDController.calculate(getDriveEncoderVelocity(),
                state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = anglePIDController.calculate(getRotationEncoder(), state.angle.getDegrees());

        final double driveFeedforward = (state.speedMetersPerSecond * 2.6);

        angleMotor.setVoltage(turnOutput);
        driveMotor.setVoltage((driveOutput + driveFeedforward) );
    }

    /**
     * stops all power to motors
     */
    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    /**
     * sets both drive and angle motor to neutralModeValue
     * 
     * @param neutralModeValue the NeutralModeValue to set the motors to. Usually
     *                         Brake or Coast
     */
    public void setModuleNeutralMode(NeutralModeValue neutralModeValue) {
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

        public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }


}
