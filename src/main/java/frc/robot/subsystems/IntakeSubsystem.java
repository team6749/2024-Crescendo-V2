// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax indexerSpark = new CANSparkMax(Constants.ElectronicsPorts.indexerSpark, MotorType.kBrushed);
    DigitalInput indexerSwitch = new DigitalInput(Constants.ElectronicsPorts.indexerSwitch);

    TalonFX intakeMotor = new TalonFX(Constants.ElectronicsPorts.intakeMotor);
    DigitalInput intakeSwitch = new DigitalInput(Constants.ElectronicsPorts.intakeSwitch);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        intakeMotor.set(0);
        indexerSpark.set(0);
        // This method will be called once per scheduler run
    }

    public void indexNote(boolean reverse) {
        if(!indexerSwitch.get()){
            if(!reverse){
            indexerSpark.set(0.5);
            }else if(reverse){
                indexerSpark.set(-0.5);
            }
        }
    }

    public void intake(boolean reverse, double voltage) {
        if (!intakeSwitch.get()) {
            if (!reverse) {
                intakeMotor.set(voltage);
            } else if (reverse) {
                intakeMotor.set(-voltage);
            }
        }
    }

    public Command indexCommand(boolean reverse, boolean load) {
        return run(() -> indexNote(reverse)); // ERM what is this and will it work
    }

    public Command intakeCommand(boolean reverse, double voltage) {
        return run(() -> intake(reverse, voltage));
    }
}
