// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    //Initializes motor variables to be declared later
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;


    //Initializing variables to control the shooters voltage and modifiers
    private double topShooterMaxModifier = 1;
    private double bottomShooterMaxModifier = 1;
    double leftvoltage = 6;
    double rightVoltage = 6;

    public ShooterSubsystem() {
        //Initializes motors with their respective ports from the ElectronicsPorts sub-class in constants
        topShooterMotor = new TalonFX(Constants.ElectronicsPorts.topShooterMotorPort);
        bottomShooterMotor = new TalonFX(Constants.ElectronicsPorts.bottomShooterMotorPort);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //Initializes a sendable builder which puts data to shuffleboard and allows users to set variables in
        //shuffleboard without having to re-deploy the code
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addDoubleProperty("Top Shooter limiter", this::getTopShooterMaxModifier,
                this::setTopShooterMaxModifier);
        builder.addDoubleProperty("Bottom shooter limiter", this::getBottomShooterMaxModifier,
                this::setBottomShooterMaxModifier);
        builder.addDoubleProperty("Shooter left voltage input", this::getLeftVoltage, this::setLeftVoltage);
        builder.addDoubleProperty("shooter right voltage input", this::getRightVoltage, this::setRightVoltage);

    }

    //Getters and setters for the shooter motors max voltage and modifiers
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
    public void shoot(double voltage) {
        leftvoltage = voltage;
        rightVoltage = voltage;
        topShooterMotor.setVoltage(leftvoltage);
        bottomShooterMotor.setVoltage(rightVoltage);

    }

    /**
     * sets both shooter motors to a reverse voltage for intaking
     */
    public void shooterIntake() {
        topShooterMotor.setVoltage(-2);
        bottomShooterMotor.setVoltage(-2);
    }

    /**
     * Sets the shooter motors to reverse input so that it can intake from the
     * source
     * 
     * @return instant command that reverses shooter motors temporarily
     */
    public Command shooterIntakeCommand() {
        return run(() -> shooterIntake());
    }

    /**
     * sets both shooter motors to a certain amount of voltage
     * 
     */
    public void shootSpeaker() {
        topShooterMotor.setVoltage(6);
        bottomShooterMotor.setVoltage(6);
    }

    /**
     * 
     * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
     * 
     * @return an instant command to set the motors to a voltage to shoot into the speaker
     */
    // builds a command in the subsystem to reduce amount of files in the repository
    // uses runOnce since when the command is called it is called using the
    // whileTrue parameter for a button
    public Command shootSpeakerCommand() {
        return runOnce(() -> shootSpeaker());
    }
}
