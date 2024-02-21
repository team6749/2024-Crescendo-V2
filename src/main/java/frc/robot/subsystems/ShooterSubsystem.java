// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;

    public ShooterSubsystem(int topShooterMotorPort, int bottomShooterMotorPort) {
        topShooterMotor = new TalonFX(topShooterMotorPort);
        bottomShooterMotor = new TalonFX(bottomShooterMotorPort);
        // i have to pee rn zac!!!
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * sets both shooter motors to a certain amount of voltage
     * 
     * @param voltage the target voltage or the motor
     */
    public void shootSpeaker(int voltage) {
        topShooterMotor.setVoltage(voltage); // TODO ask zac if this is right
        bottomShooterMotor.setVoltage(voltage); // TODO ask zac if this is right
    }

    /**
     * 
     * https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
     * @param voltage the target voltage
     * @return an instant command to set the motors to voltage
     */
    public Command shootSpeakerCommand(int voltage) {
        return runOnce(() -> shootSpeaker(voltage)); // ERM what is this and will it work
    }
}
