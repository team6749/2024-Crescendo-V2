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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    // Defining electronics with their associated device ID's in Constants
    CANSparkMax indexerSpark = new CANSparkMax(Constants.ElectronicsPorts.indexerSpark,
            CANSparkLowLevel.MotorType.kBrushed);

    TalonFX intakeMotor = new TalonFX(Constants.ElectronicsPorts.intakeMotor);
    private DigitalInput intakeSwitch = new DigitalInput(Constants.ElectronicsPorts.intakeSwitch);

    double indexerVoltage = 0;
    double intakeVoltage = 0;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        indexerSpark.setIdleMode(IdleMode.kBrake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);
        builder.addBooleanProperty("limit switch", () -> getLimitSwitch(), null);
    }
    

    @Override
    public void periodic() {
        // Default set motors to 0 power so that they do not run randomly
        intakeMotor.setVoltage(intakeVoltage);
        indexerSpark.setVoltage(indexerVoltage);
    }

    /**
     * @param volts volts to run the indexer at
     * @return instantly starts the indexer motors to get the game piece to a
     *         desired spot in the robot
     */
    public void indexNote(double volts) {
        indexerVoltage = volts;
    }

    /**
     * 
     * @param reverse boolean to deicide to intake normally or to spit game piece
     *                out bottom
     * @param voltage desired voltage to run intake motors at
     * @return instantly runs intake motors to intake game piece from the ground
     */
    public void intake(double voltage) {
        this.intakeVoltage = voltage;
    }

    /**
     * 
     * @return the state of the limit switch :)
     */
    public boolean getLimitSwitch() {
        return intakeSwitch.get();
    }

    /**
     * Sets indexer motors power to zero
     */
    public void stopIndexer() {
        indexerVoltage = 0;
    }

    /**
     * sets intake motors power to zero
     */
    public void stopIntake() {
        intakeVoltage = 0;
    }

    public Command groundIntake() {
        return Commands.runEnd(
                () -> {
                    if (getLimitSwitch() == false) {
                        intake(-1);
                        indexNote(6);
                    } else {
                        stopIndexer();
                        stopIntake();
                    }
                },
                () -> {
                    System.out.println("ended intake command");
                    stopIndexer();
                    stopIntake();
                }, this);
    }

}
