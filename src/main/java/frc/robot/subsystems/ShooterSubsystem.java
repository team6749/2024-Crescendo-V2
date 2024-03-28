// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    // Initializes motor variables to be declared later
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;

    // Initializing variables to control the shooters voltage and modifiers
    private double topShooterMaxModifier = 1;
    private double bottomShooterMaxModifier = 1;
    double voltage = 0;

    public ShooterSubsystem() {
        // Initializes motors with their respective ports from the ElectronicsPorts
        // sub-class in constants
        topShooterMotor = new TalonFX(Constants.ElectronicsPorts.topShooterMotorPort);
        bottomShooterMotor = new TalonFX(Constants.ElectronicsPorts.bottomShooterMotorPort);
    }

    @Override
    public void periodic() {
        topShooterMotor.setVoltage(voltage * topShooterMaxModifier);
        bottomShooterMotor.setVoltage(voltage * bottomShooterMaxModifier);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // Initializes a sendable builder which puts data to shuffleboard and allows
        // users to set variables in
        // shuffleboard without having to re-deploy the code
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addDoubleProperty("Top Shooter limiter", this::getTopShooterMaxModifier,
                this::setTopShooterMaxModifier);
        builder.addDoubleProperty("Bottom shooter limiter", this::getBottomShooterMaxModifier,
                this::setBottomShooterMaxModifier);
        builder.addDoubleProperty("Shooter voltage", this::getVoltage, this::setVotlage);

    }

    /**
     * gets the desired voltage amount for rapid testing, used for getter on a
     * sendable builder
     * 
     * @return the desired voltage for both motors
     */
    public double getVoltage() {
        return voltage;
    }

    /**
     * passes through the desired voltage amount for rapid testing, used for setter
     * on a sendable builder
     * 
     * @param voltage
     */
    public void setVotlage(double voltage) {
        this.voltage = voltage;
    }

    /**
     * gets the desired modifier amount for rapid testing, used for getter on a
     * sendable builder
     * 
     * @return the desired modifier for the top shooter
     */
    public double getTopShooterMaxModifier() {
        return topShooterMaxModifier;
    }

    /**
     * passes through the desired modifier amount for rapid testing, used for setter
     * on a sendable builder
     * 
     * @param topShooterMaxModifier the desired modifier for the top shooter
     */
    public void setTopShooterMaxModifier(double topShooterMaxModifier) {
        this.topShooterMaxModifier = topShooterMaxModifier;
    }

    /**
     * gets the desired modifier amount for rapid testing, used for getter on a
     * sendable builder
     * 
     * @return the desired modifier for the bottom shooter
     */
    public double getBottomShooterMaxModifier() {
        return bottomShooterMaxModifier;
    }

    /**
     * passes through the desired modifier amount for rapid testing, used for setter
     * on a sendable builder
     * 
     * @param bottomShooterMaxModifier the desired modifier for the bottom shooter
     */
    public void setBottomShooterMaxModifier(double bottomShooterMaxModifier) {
        this.bottomShooterMaxModifier = bottomShooterMaxModifier;
    }

    /**
     * sets both shooter motors to a certain amount of voltage as specified by user
     * 
     * @param voltage        the target voltage for the motor
     * @param topModifier    the % amount of input voltage to apply to the top
     *                       shooter
     * @param bottomModifier the % amount of input voltage to apply to the bottom
     *                       shooter
     */
    public void shoot(double voltage, double topModifier, double bottomModifier) {
        this.voltage = voltage;
        this.bottomShooterMaxModifier = bottomModifier;
        this.topShooterMaxModifier = topModifier;
    }

}
