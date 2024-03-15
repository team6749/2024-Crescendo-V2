// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX climber = new TalonFX(Constants.ElectronicsPorts.climberMotor);
    DigitalInput climberSwitch = new DigitalInput(Constants.ElectronicsPorts.climberSwitch);
    boolean climberHomed = false;

    boolean isClimberEnabled = false;
    double targetVoltage;
    Debouncer sensorlessHomerDebounce = new Debouncer(0.05, Debouncer.DebounceType.kRising);
    

    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
        climber.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (RobotState.isEnabled()) {
            if (getBottomLimitSwitch() && climberHomed == false) {
                climberHomed = true;
                setClimberPosition(0.01);
                climber.stopMotor();
            }
            if (climberHomed == false) {
                climber.setVoltage(0.5);
                return;
            }
        }
        if ((getClimberPosition() < 0 || getBottomLimitSwitch()) && targetVoltage > 0) {
            climber.stopMotor();
            return;
        }
        if (getClimberPosition() > 0.5 && targetVoltage < 0) {
            climber.stopMotor();
            return;
        }
        if (isClimberEnabled) {
            climber.setVoltage(targetVoltage);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("limit switch", () -> getBottomLimitSwitch(), null);
        builder.addBooleanProperty("is homed", () -> climberHomed, null);
        builder.addDoubleProperty("Climber voltage", () -> targetVoltage, this::setTargetVoltage);
        builder.addDoubleProperty("Climber velocity", this::getClimberVelocity, null);
        builder.addDoubleProperty("Climber Position", this::getClimberPosition, null);
        builder.addDoubleProperty("Climber actual voltage%", () -> climber.getMotorVoltage().getValueAsDouble(), null);
        builder.setSafeState(this::stop);
    }

    public Boolean getBottomLimitSwitch() {
        return (sensorlessHomerDebounce.calculate(climber.getMotorVoltage().getValueAsDouble() > 0) && getClimberVelocity() > -0.004);
    }

    public void setTargetVoltage(double voltage) {
        this.targetVoltage = voltage;
    }

    public void start() {
        isClimberEnabled = true;
    }

    public void stop() {
        isClimberEnabled = false;
        climber.stopMotor();
    }

    public double getClimberPosition() {
        return -(climber.getPosition().getValueAsDouble() * Constants.conversionConstants.climberConversion);
    }

    private void setClimberPosition(double newPosition) {
        climber.setPosition(-(newPosition / Constants.conversionConstants.climberConversion));
    }

    public double getClimberVelocity() {
        return -(climber.getVelocity().getValueAsDouble() * Constants.conversionConstants.climberConversion);
    }

    public Command raiseClimber() {
        return Commands.startEnd(
                () -> {
                    setTargetVoltage(-3);
                    start();
                },
                () -> {
                    stop();
                }, this);
    }

    public Command lowerClimber() {
        return Commands.startEnd(
                () -> {
                    setTargetVoltage(3);
                    start();
                },
                () -> {
                    stop();
                }, this);
    }

}
