// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX climber = new TalonFX(Constants.ElectronicsPorts.climberMotor);
    DigitalInput climberSwitch = new DigitalInput(Constants.ElectronicsPorts.climberSwitch);
    boolean climberDown = true;
    


    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
        climber.setNeutralMode(NeutralModeValue.Brake);
        climber.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("limit switch", () -> getClimberSwitch(), null);
        builder.addBooleanProperty("is down", () -> climberDown, null);
    }

    public Boolean getClimberSwitch() {
        return climberSwitch.get();
    }

    public void climberUp(){
        climber.setVoltage(4);
    }
    public void climberDown(){
        // if(!getClimberSwitch()){
        // climber.setVoltage(0);
        // }
        // else{
            climber.setVoltage(-4);
        // }  
    }
    public void climberStop(){
        climber.setVoltage(0);
    }

public Command raiseClimber(){
    return Commands.startEnd(
        ()-> {
            if(climberDown == true){
            climberUp();
            }
        }, 
        ()-> {
            climberStop();
            climberDown = false;
        }, this);
}

public Command lowerClimber(){
    return Commands.startEnd(
        ()-> {
            if(climberDown == false){
            climberDown();
            }
        },
        ()-> {
            climberStop();
            climberDown = true;
        }, this)/* .until(()-> getClimberSwitch())*/;
}
}
