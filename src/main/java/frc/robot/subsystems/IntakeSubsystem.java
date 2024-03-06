// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //Defining electronics with their associated device ID's in Constants
    CANSparkMax indexerSpark = new CANSparkMax(Constants.ElectronicsPorts.indexerSpark, CANSparkLowLevel.MotorType.kBrushed);

    TalonFX intakeMotor = new TalonFX(Constants.ElectronicsPorts.intakeMotor);
    DigitalInput intakeSwitch = new DigitalInput(Constants.ElectronicsPorts.intakeSwitch);



    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        //Default set motors to 0 power so that they do not run randomly
    }

    /**
     * 
     * @param reverse boolean to decide whether to intake normally or to spit the game piece out the bottom
     * @param override boolean to override the limit switch for shooting
     * @return instantly starts the indexer motors to get the game piece to a desired spot in the robot
     */
    public void indexNote(boolean reverse, boolean override) {
        if(!intakeSwitch.get() ){
            if(!reverse){
            indexerSpark.setVoltage(8);
            }else if(reverse){
                indexerSpark.setVoltage(8);
            }
        }if(override){
            indexerSpark.setVoltage(8);
        }
    }
    
    /**
     * 
     * @param reverse boolean to deicide to intake normally or to spit game piece out bottom
     * @param voltage desired voltage to run intake motors at
     * @return instantly runs intake motors to intake game piece from the ground
     */
    public void intake(boolean reverse, double voltage) {
        if(!intakeSwitch.get()) {
            if (!reverse) {
                intakeMotor.setVoltage(voltage);
            } else if (reverse) {
                intakeMotor.setVoltage(-voltage);
            }
        }
    }

    /**
     * Sets indexer motors power to zero
     */
    public void stopIndexer(){
        indexerSpark.set(0);
    }
    /**
     * sets intake motors power to zero
     */
    public void stopIntake(){
        intakeMotor.set(0);
    }
    
}
