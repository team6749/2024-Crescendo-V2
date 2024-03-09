// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    // Initializes motor variables to be declared later
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;

    // Initializing variables to control the shooters voltage and modifiers
    private double topShooterMaxModifier = 1;
    private double bottomShooterMaxModifier = 1;
    double leftvoltage = 0;
    double rightVoltage = 0;

    public ShooterSubsystem() {
        // Initializes motors with their respective ports from the ElectronicsPorts
        // sub-class in constants
        topShooterMotor = new TalonFX(Constants.ElectronicsPorts.topShooterMotorPort);
        bottomShooterMotor = new TalonFX(Constants.ElectronicsPorts.bottomShooterMotorPort);
    }

    @Override
    public void periodic() {
        topShooterMotor.setVoltage(leftvoltage * topShooterMaxModifier);
        bottomShooterMotor.setVoltage(rightVoltage * bottomShooterMaxModifier);

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
        builder.addDoubleProperty("Shooter left voltage input", this::getLeftVoltage, this::setLeftVoltage);
        builder.addDoubleProperty("shooter right voltage input", this::getRightVoltage, this::setRightVoltage);

    }

    // Getters and setters for the shooter motors max voltage and modifiers
    public double getRightVoltage() {
        return rightVoltage;
    }

    public void setRightVoltage(double rightVoltage) {
        this.rightVoltage = rightVoltage;
    }

    public double getLeftVoltage() {
        return leftvoltage;
    }

    public void setLeftVoltage(double voltage) {
        this.leftvoltage = voltage;
    }

    public double getTopShooterMaxModifier() {
        return topShooterMaxModifier;
    }

    public void setTopShooterMaxModifier(double topShooterMaxModifier) {
        this.topShooterMaxModifier = topShooterMaxModifier;
    }

    public double getBottomShooterMaxModifier() {
        return bottomShooterMaxModifier;
    }

    public void setBottomShooterMaxModifier(double bottomShooterMaxModifier) {
        this.bottomShooterMaxModifier = bottomShooterMaxModifier;
    }

    /**
     * sets both shooter motors to a certain amount of voltage
     * 
     * @param voltage the target voltage for the motor
     */
    public void shoot(double voltage, double topModifier, double bottomModifier) {
        leftvoltage = voltage;
        rightVoltage = voltage;
        this.bottomShooterMaxModifier = bottomModifier;
        this.topShooterMaxModifier = topModifier;
    }

}
