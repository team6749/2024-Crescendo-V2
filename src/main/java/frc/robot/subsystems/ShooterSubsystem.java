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
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;

    private double topShooterMaxModifier = 1;
    private double bottomShooterMaxModifier = 1;
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

    public ShooterSubsystem() {
        topShooterMotor = new TalonFX(Constants.ElectronicsPorts.topShooterMotorPort);
        bottomShooterMotor = new TalonFX(Constants.ElectronicsPorts.bottomShooterMotorPort);
        // i have to pee rn zac!!!
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        topShooterMotor.setVoltage(0);
        bottomShooterMotor.setVoltage(0);
    }
    @Override
	public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addDoubleProperty("Top Shooter limiter", this::getTopShooterMaxModifier, this::setTopShooterMaxModifier);
        builder.addDoubleProperty("Bottom shooter limiter", this::getBottomShooterMaxModifier, this::setBottomShooterMaxModifier);
        
    }

    /**
     * sets both shooter motors to a certain amount of voltage
     * 
     * @param voltage the target voltage or the motor
     */
    public void shoot(int voltage) {
        topShooterMotor.setVoltage(voltage * topShooterMaxModifier);
        bottomShooterMotor.setVoltage(voltage* bottomShooterMaxModifier); //TODO find out which one is negative voltage
    }

    /**
     * 
     * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
     * @param voltage the target voltage
     * @return an instant command to set the motors to voltage
     */
    public Command shootCommand(int voltage) {
        return run(() -> shoot(voltage)); // ERM what is this and will it work
    }

    /**
     * sets both shooter motors to a certain amount of voltage
     * 
     * @param voltage the target voltage or the motor
     */
    public void shootSpeaker(int voltage) {
        topShooterMotor.setVoltage(voltage); // TODO ask zac if this is right
        bottomShooterMotor.setVoltage(voltage); // TODO ask zac if this is right
    }

    /**
     * 
     * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
     * @param voltage the target voltage
     * @return an instant command to set the motors to voltage
     */
    public Command shootSpeakerCommand(int voltage) {
        return runOnce(() -> shootSpeaker(voltage)); // ERM what is this and will it work
    }
}
