// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax intakeSpark = new CANSparkMax(Constants.ElectronicsPorts.intakeSpark, MotorType.kBrushed);
    DigitalInput intakeSwitch = new DigitalInput(Constants.ElectronicsPorts.intakeSwitch);
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        intakeSpark.set(0);
        // This method will be called once per scheduler run
    }
    public void runIntake(boolean reverse){
        if(intakeSwitch.get() != true){
            if(reverse){
                intakeSpark.set(-0.5);
            }else if(reverse != true){
                intakeSpark.set(0.5);
            }
        }
    }
}
