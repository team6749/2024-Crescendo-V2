// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;

    private double topShooterMaxModifier = 1;
    private double bottomShooterMaxModifier = 1;
    double voltage = 0;
    boolean isShooting = false;
    double shooterVelocityMs = 0;
    double peakVelocity = 0;
    public boolean noteHasLeft = false;

    public ShooterSubsystem() {
        topShooterMotor = new TalonFX(Constants.ElectronicsPorts.topShooterMotorPort);
        bottomShooterMotor = new TalonFX(Constants.ElectronicsPorts.bottomShooterMotorPort);
    }
    
    @Override
    public void periodic() {
        topShooterMotor.setVoltage(voltage * topShooterMaxModifier);
        bottomShooterMotor.setVoltage(voltage * bottomShooterMaxModifier);

        shooterVelocityMs = ((bottomShooterMotor.getVelocity().getValueAsDouble() + topShooterMotor.getVelocity().getValueAsDouble()) / 2) * 0.0762 * Math.PI;
        
        if(voltage != 0) {
            if(shooterVelocityMs > peakVelocity) {
                peakVelocity = shooterVelocityMs;
            }
            if(shooterVelocityMs < peakVelocity * 0.9 && noteHasLeft == false) {
                noteHasLeft = true;
                System.out.println("note left shooter");
            }
        } else {
            peakVelocity = 0;
            noteHasLeft = false;
        }
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
        builder.addDoubleProperty("Velocity", () -> shooterVelocityMs, null);
    }
    
    /**
     * gets the desired voltage amount for rapid testing, used for getter on a
     * sendable builder
     * 
     * @return the desired voltage for both motors
     */
    public double getVoltage(){
        return voltage;
    }
    
    /**
     * passes through the desired voltage amount for rapid testing, used for setter on a sendable builder
     * @param voltage
     */
    public void setVotlage(double voltage){
        this.voltage = voltage;
    }
    
    public boolean isShooting() {
        return isShooting;
    }

    public void setShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }
    
   /**
    * gets the desired modifier amount for rapid testing, used for getter on a sendable builder
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
