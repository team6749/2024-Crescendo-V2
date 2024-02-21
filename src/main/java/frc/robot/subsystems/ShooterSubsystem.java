// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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

}
