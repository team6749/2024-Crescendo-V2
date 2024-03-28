// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LightsCommand extends Command {
    ClimberSubsystem _climber;
    IntakeSubsystem _intake;
    ShooterSubsystem _shooter;
    LightsSubsystem _lights;

    /** Creates a new LightsCommand. */
    public LightsCommand(LightsSubsystem lights, ClimberSubsystem climber, ShooterSubsystem shooter,
            IntakeSubsystem intake) {
        _climber = climber;
        _lights = lights;
        _shooter = shooter;
        _intake = intake;
        addRequirements(lights);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_climber.isCurrentlyClimbing()) { // works
            _lights.magenta();
        } else if (_shooter.isShooting()) { // works
            _lights.cyan();
        } else if (_intake.getNoteDetected()) { // works
            _lights.green();
        } else if(_climber.isAmplify() == true){
            _lights.amplificationLights();
        }else if (!_intake.getNoteDetected() && !_shooter.isShooting() && !_climber.isCurrentlyClimbing() && !_lights.isAmplify() && !_lights.isCoopertition()){
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                _lights.blue();
            } else {
                _lights.red();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
