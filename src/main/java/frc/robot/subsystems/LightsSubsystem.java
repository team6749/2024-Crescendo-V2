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

    public Command setLightsCommand(int no) {
        return Commands.runOnce(
        () -> {
        
        switch (no) {
            case (1): 
                green();
                break;
            case (2): 
                red();
                break;
            case (3):
                blue();
                break;
            case (4):
                magenta();
                break;
            case (5):
                yellow();
                break;
            case (6):
                cyan();
                break;
            case (7):
                white();
                break;
            default: 
                off();
                break;
        }}
        ,
        this);
    }
    public Command rainbowLights(){
        return Commands.repeatingSequence(
            setLightsCommand(1),
            Commands.waitSeconds(0.2),
            setLightsCommand(2),
            Commands.waitSeconds(0.2),
            setLightsCommand(3),
            Commands.waitSeconds(0.2),
            setLightsCommand(4),
            Commands.waitSeconds(0.2),
            setLightsCommand(5),
            Commands.waitSeconds(0.2),
            setLightsCommand(6),
            Commands.waitSeconds(0.2),
            setLightsCommand(7),
            Commands.waitSeconds(0.2)
        );
    }
}