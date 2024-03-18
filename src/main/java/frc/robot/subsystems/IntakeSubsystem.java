// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    // Defining electronics with their associated device ID's in Constants
    TalonFX indexerMotor = new TalonFX(Constants.ElectronicsPorts.indexerFalcon);

    TalonFX intakeMotor = new TalonFX(Constants.ElectronicsPorts.intakeMotor);

    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    double indexerVoltage = 0;
    double intakeVoltage = 0;

    boolean isConnected = false;
    double proximity = 0;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("note dectected", () -> getNoteDetected(), null);
        builder.addDoubleProperty("proximity", () -> proximity, null);
        builder.addBooleanProperty("Color sensor working?", () -> isConnected, null);
    }
    
    @Override
    public void periodic() {
        intakeMotor.setVoltage(intakeVoltage);
        indexerMotor.setVoltage(indexerVoltage);
        proximity = colorSensor.getProximity();
        isConnected = true;
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
     * @return is the note within range of the color sensor
     */
    public boolean getNoteDetected() {
        return proximity > 500;
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

    /**
     * runs the intake and indexer to pick up notes and feed them into the robot
     * @return stops intaking if not is detected and then sets indexer and intake power to 0 volts
     */
    public Command groundIntake() {
        return Commands.runEnd(
                () -> {
                    if (getNoteDetected() == false) {
                        intake(-1);
                        indexNote(6);
                    } else {
                        stopIndexer();
                        stopIntake();
                    }
                },
                () -> {
                    // System.out.println("ended intake command");
                    stopIndexer();
                    stopIntake();
                }, this);
    }




}
