// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
    }


    private static Relay green = new Relay(Constants.ElectronicsPorts.green);
    private static Relay red = new Relay(Constants.ElectronicsPorts.red);
    private static Relay blue = new Relay(Constants.ElectronicsPorts.blue);

    public DriverStation.Alliance getAlliance() { // TO TEST
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red || ally.get() == Alliance.Blue) {
                return ally.get();
            }
        }
        return null;
    }

    // public void setAllianceColors() {
    //     if (getAlliance() == Alliance.Red) {
    //         Red();
    //     }
    //     if (getAlliance() == Alliance.Blue) {
    //         Blue();
    //     }
    // }

    

    @Override
    public void periodic() {}

    public Command rainbowLights(){
        return Commands.repeatingSequence(
            greenCommand(),
            Commands.waitSeconds(0.2),
            cyanCommand(),
            Commands.waitSeconds(0.2),
            redCommand(),
            Commands.waitSeconds(0.2),
            blueCommand(),
            Commands.waitSeconds(0.2),
            whiteCommand(),
            Commands.waitSeconds(0.2),
            yellowCommand(),
            Commands.waitSeconds(0.2),
            magentaCommand(),
            Commands.waitSeconds(0.2)
        );
    }


    public Command greenCommand(){
        return Commands.run(        
    ()-> {
        green.set(Relay.Value.kOff);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    },
         this);
    }
    public Command cyanCommand(){
        return Commands.run(        
    ()-> {
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    },
         this);
    }
    public Command redCommand(){
        return Commands.run(        
    ()-> {
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kOff);
    },
         this);
    }
    public Command blueCommand(){
        return Commands.run(        
    ()-> {
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    },
         this);
    }
    public Command whiteCommand(){
        return Commands.run(        
    ()-> {
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    },
         this);
    }
    public Command yellowCommand(){
        return Commands.run(        
    ()-> {
        green.set(Value.kOff);
        blue.set(Value.kForward);
        red.set(Value.kOff);
    },
         this);
    }
    public Command magentaCommand(){
        return Commands.run(        
    ()-> {
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    },
         this);
    }
    public Command offCommand(){
        return Commands.run(        
    ()-> {
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    },
         this);
    }
}