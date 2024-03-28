// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
    int teamColor; // 1 is blue, 2 is red
    boolean amplify = false;
    boolean coopertition = false;
    Timer timer = new Timer();

    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
        timer.reset();
    }

    private static Relay green = new Relay(Constants.ElectronicsPorts.green);
    private static Relay red = new Relay(Constants.ElectronicsPorts.red);
    private static Relay blue = new Relay(Constants.ElectronicsPorts.blue);

    
    @Override
    public void periodic(){
        if (DriverStation.getAlliance().get().toString() == "Blue") {
            teamColor = 1;
        } else if (DriverStation.getAlliance().get().toString() == "Red") {
            teamColor = 2;
        }
    }

    public boolean isAmplify() {
        return amplify;
    }

    public void setAmplify(boolean amplify) {
        this.amplify = amplify;
        System.out.println(this.amplify);
    }

    public boolean isCoopertition() {
        return coopertition;
    }

    public void setCoopertition(boolean coop) {
        this.coopertition = coop;
    }

    public void off(){
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    }
    public void cyan() {
        System.out.println("cyan");
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    }

    public void green() {
        System.out.println("green");
        green.set(Relay.Value.kOff);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kForward);
    }

    public void red() {
        System.out.println("red");
        green.set(Relay.Value.kForward);
        blue.set(Relay.Value.kForward);
        red.set(Relay.Value.kOff);
    }

    public void blue() {
        System.out.println("blue");
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kForward);
    }

    public void white() {
        System.out.println("white");
        green.set(Value.kOff);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    }

    public void yellow() {
        System.out.println("yellow");
        green.set(Value.kOff);
        blue.set(Value.kForward);
        red.set(Value.kOff);
    }

    public void magenta() {
        System.out.println("magenta");
        green.set(Value.kForward);
        blue.set(Value.kOff);
        red.set(Value.kOff);
    }

    public void amplificationLights() {
        timer.start();
        if(teamColor == 1){
            for (int i = 0; i < 100; i++) {
                blue();
                off();
                }
            
        }else if(teamColor == 2){
            for (int i = 0; i < 100; i++) {
                red();
                off();
            }
        }else {
            green();
        }
    }

    public Command rainbowLights() {
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
                Commands.waitSeconds(0.2)).withTimeout(10);
    }

    public Command defaultColorCommand() {
        return Commands.runOnce(
                () -> {
                    if (teamColor == 1) {
                        blueCommand();
                    } else {
                        greenCommand();
                    }
                },
                this);
    }

    public Command greenCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Relay.Value.kOff);
                    blue.set(Relay.Value.kForward);
                    red.set(Relay.Value.kForward);
                },
                this);
    }

    public Command cyanCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Value.kOff);
                    blue.set(Value.kOff);
                    red.set(Value.kForward);
                },
                this);
    }

    public Command redCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Relay.Value.kForward);
                    blue.set(Relay.Value.kForward);
                    red.set(Relay.Value.kOff);
                },
                this);
    }

    public Command blueCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Value.kForward);
                    blue.set(Value.kOff);
                    red.set(Value.kForward);
                },
                this);
    }

    public Command whiteCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Value.kOff);
                    blue.set(Value.kOff);
                    red.set(Value.kOff);
                },
                this);
    }

    public Command yellowCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Value.kOff);
                    blue.set(Value.kForward);
                    red.set(Value.kOff);
                },
                this);
    }

    public Command magentaCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Value.kForward);
                    blue.set(Value.kOff);
                    red.set(Value.kOff);
                },
                this);
    }

    public Command offCommand() {
        return Commands.runOnce(
                () -> {
                    green.set(Relay.Value.kForward);
                    blue.set(Relay.Value.kForward);
                    red.set(Relay.Value.kForward);
                },
                this);
    }
}