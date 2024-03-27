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
    Alliance teamColor;
    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
    }


    private static Relay green = new Relay(Constants.ElectronicsPorts.green);
    private static Relay red = new Relay(Constants.ElectronicsPorts.red);
    private static Relay blue = new Relay(Constants.ElectronicsPorts.blue);

    public void getRobotAlliance() { // TO TEST
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red || ally.get() == Alliance.Blue) {
                teamColor = ally.get();
            }
        }
    }


    public void off(){
        System.out.println("off");
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    }
    public void cyan(){
        System.out.println("cyan");
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    }
    public void green(){
        System.out.println("green");
        green.set(Relay.Value.kOff);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    }
    public void red(){
        System.out.println("red");
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kOff);
    }
    public void blue(){
        System.out.println("blue");
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    }
    public void white(){
        System.out.println("white");
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    }
    public void yellow(){
        System.out.println("yellow");
        green.set(Value.kOff);
        blue.set(Value.kForward);
        red.set(Value.kOff);
    }
    public void magenta(){
        System.out.println("magenta");
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    }

    

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


    public Command defaultColorCommand(){
        return Commands.runOnce(
    ()-> {
        if(teamColor == DriverStation.Alliance.Blue){
            blueCommand();
        }else{
            greenCommand();
        }
    }, 
    this);
    }
    
    public Command greenCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Relay.Value.kOff);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    },
         this);
    }
    public Command cyanCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    },
         this);
    }
    public Command redCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kOff);
    },
         this);
    }
    public Command blueCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    },
         this);
    }
    public Command whiteCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    },
         this);
    }
    public Command yellowCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Value.kOff);
        blue.set(Value.kForward);
        red.set(Value.kOff);
    },
         this);
    }
    public Command magentaCommand(){
        return Commands.runOnce(  
    ()-> {
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    },
         this);
    }
    public Command offCommand(){
        return Commands.runOnce(        
    ()-> {
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    },
         this);
    }
}