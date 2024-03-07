// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //Defining electronics with their associated device ID's in Constants
    CANSparkMax indexerSpark = new CANSparkMax(Constants.ElectronicsPorts.indexerSpark, CANSparkLowLevel.MotorType.kBrushed);

    TalonFX intakeMotor = new TalonFX(Constants.ElectronicsPorts.intakeMotor);
    DigitalInput intakeSwitch = new DigitalInput(Constants.ElectronicsPorts.intakeSwitch);

    double indexerVoltage = 0;
    double intakeVoltage = 0;



    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        indexerSpark.setIdleMode(IdleMode.kBrake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        //Default set motors to 0 power so that they do not run randomly
        intakeMotor.setVoltage(intakeVoltage);
        indexerSpark.setVoltage(indexerVoltage);    
        SmartDashboard.putBoolean("Intake limit switch", intakeSwitch.get());
    }

    /**
     * 
     * @param reverse boolean to decide whether to intake normally or to spit the game piece out the bottom
     * @param override boolean to override the limit switch for shooting
     * @return instantly starts the indexer motors to get the game piece to a desired spot in the robot
     */
    public void indexNote(boolean reverse, boolean override) {
        if(!getLimitSwitch()){
            if(!reverse){
                indexerVoltage = 6.5;
            }else if(reverse){
                indexerVoltage = -6.5;
            }
        }if(override){
            indexerVoltage = 8;
        }
    }
    
    /**
     * 
     * @param reverse boolean to deicide to intake normally or to spit game piece out bottom
     * @param voltage desired voltage to run intake motors at
     * @return instantly runs intake motors to intake game piece from the ground
     */
    public void intake(boolean reverse, double voltage) {
        if(!getLimitSwitch()) {
            if (!reverse) {
                this.intakeVoltage = voltage;
            } else if (reverse) {
                this.intakeVoltage = -voltage;
            }
        }
    }

    /**
     * 
     * @return the state of the limit switch :)
     */
    public boolean getLimitSwitch(){
        return intakeSwitch.get();
    }


    /**
     * Sets indexer motors power to zero
     */
    public void stopIndexer(){
        indexerVoltage = 0;
    }
    /**
     * sets intake motors power to zero
     */
    public void stopIntake(){
        intakeVoltage = 0;
    }

    public Command intakeComand(){
        return run(()-> intake(false, 2)).finallyDo(()-> stopIntake());
    }
    public Command indexCommand(){
        return run(()->indexNote(false, false)).finallyDo(()->stopIndexer());
    }
    
}
