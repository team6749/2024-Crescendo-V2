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

@SuppressWarnings("unused")
public class SwerveModule implements Sendable {
    /** Creates a new SwerveModule. */
    public String name;
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder encoder;
    public Translation2d location;
    public final PIDController velocityPIDController = new PIDController(1.5, 0, 0);
    public final PIDController anglePIDController = new PIDController(0.1, 0, 0);
    public double maxSpeed;

    private double m_rotation = 0;
    private double m_velocity = 0;
    private double m_position = 0;
    private SwerveModuleState m_targetstate = new SwerveModuleState();

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
        builder.addStringProperty(name, this::getName, null);
        builder.addDoubleProperty("velocity", this::getModuleVelocityMs, null);
        builder.addDoubleProperty("position", this::getModulePositionM, null);
        builder.addDoubleProperty("rotation", () -> getModuleRotation().getDegrees(), null);
        builder.addDoubleProperty("Max Speed", this::getMaxSpeed, this::setMaxSpeed);
        builder.addDoubleProperty("Target Velocity", () -> m_targetstate.speedMetersPerSecond, (newValue) -> {
            m_targetstate.speedMetersPerSecond = newValue;
        });
        builder.addDoubleProperty("Target Rotation", () -> m_targetstate.angle.getDegrees(), (newValue) -> {
            m_targetstate.angle = Rotation2d.fromDegrees(newValue);
        });

        SmartDashboard.putData("swerve " + name + " velocity pid", velocityPIDController);
        SmartDashboard.putData("swerve " + name + " angle pid", anglePIDController);
        builder.setSafeState(this::stop);
    }

    public String getName() {
        return name;
    }

    public void periodic() {
        m_position = driveMotor.getPosition().getValueAsDouble() / (Constants.SwerveConstants.swerveGearRatio)
                * (Math.PI * Constants.SwerveConstants.swerveWheelDiameterMeters);
        m_velocity = driveMotor.getVelocity().getValueAsDouble() / (Constants.SwerveConstants.swerveGearRatio)
                * (Math.PI * Constants.SwerveConstants.swerveWheelDiameterMeters);
        m_rotation = encoder.getAbsolutePosition().getValueAsDouble() * 360;

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(m_targetstate,
                getModuleRotation());

        // // Calculate the drive output from the drive PID controller.
        final double driveOutput = velocityPIDController.calculate(getModuleVelocityMs(),
                state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = anglePIDController.calculate(getModuleRotation().getDegrees(),
                state.angle.getDegrees());

        final double driveFeedforward = (state.speedMetersPerSecond * 2.8);

        angleMotor.setVoltage(Math.min(turnOutput, Constants.SwerveConstants.turnMotorMaxOutputVolts));
        driveMotor.setVoltage((driveOutput + driveFeedforward));
    }

    /**
     * @return the velocity of robot in meters per second
     */
    public double getModuleVelocityMs() {
        return m_velocity;
    }

    /**
     * @return the position of the drive motor
     */
    public double getModulePositionM() {
        return m_position;

    }

    /**
     * @return the rotation of a module in degrees
     */
    public Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees(m_rotation);
    }

    /**
     * @return the SwerveModuleState of the module - contains the velocity and
     *         rotation of the module
     *         see
     *         https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     *         for more info
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleVelocityMs(), getModuleRotation());
    }

    /**
     * @return the position of the module
     *         see
     *         https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
     *         for more info
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getModulePositionM(), getModuleRotation());

    }

    /**
     * sets the swerve module to the desiredState
     * 
     * @param desiredState the new desired state of a SwerveModule
     */
    public void setSwerveModuleState(SwerveModuleState desiredState) {
        m_targetstate = desiredState;
    }

    /**
     * stops all power to motors
     */
    public void stop() {
        m_targetstate = new SwerveModuleState();
        driveMotor.stopMotor();
        angleMotor.stopMotor();
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
