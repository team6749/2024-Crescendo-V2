// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.NotSerializableException;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase {
    // Defining electronics with their associated device ID's in Constants
    TalonFX indexerMotor = new TalonFX(Constants.ElectronicsPorts.indexerFalcon);

    TalonFX intakeMotor = new TalonFX(Constants.ElectronicsPorts.intakeMotor);

    DigitalInput noteSensor = new DigitalInput(Constants.ElectronicsPorts.noteSensorPort);

    Debouncer debounce = new Debouncer(0.2);

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
        builder.addBooleanProperty("note dectected", () -> isBeamBreakTriggered(), null);
        builder.addDoubleProperty("proximity", () -> proximity, null);
        builder.addBooleanProperty("Is note sensor having note", () -> noteSensor.get(), null);
    }

    @Override
    public void periodic() {
        intakeMotor.setVoltage(intakeVoltage);
        indexerMotor.setVoltage(indexerVoltage);
        // colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes10bit, ProximitySensorMeasurementRate.kProxRate100ms);
        
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
     * Uses a debounce delay to calculate how long the note has been in
     * @return True if the note is in the robot for long enough (based on the debouncer) and ready to shoot
     */
    public boolean isNoteIn(){
        return debounce.calculate(isBeamBreakTriggered());
    }

    /**
     * 
     * @param voltage desired voltage to run intake motors at
     * @return instantly runs intake motors to intake game piece from the ground
     */
    public void intake(double voltage) {
        this.intakeVoltage = voltage;
    }

    /**
     * 
     * @return is the note within range of the Beam Break Sensor
     */
    public boolean isBeamBreakTriggered() {
        return !noteSensor.get();
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
     * 
     * Because of how we format our pathplanner autos are formatted (ie: pathfollow and intake race group),
     * The command is written to NEVER END
     * 
     * @return stops intaking if not is detected and then sets indexer and intake
     *         power to 0 volts
     */
    public Command groundIntake() {
        return Commands.runEnd(
                () -> {
                    if (isNoteIn() == false) { // if no note in, try to intake    
                        intake(-2);
                        indexNote(7);
                    } else { // otherwise, dont run the intake or indexer
                        stopIndexer();
                        stopIntake();
                    }
            
                    },
                () -> {
                    stopIndexer();
                    stopIntake();
                }, this);
    }

}
