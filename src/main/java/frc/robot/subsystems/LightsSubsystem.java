// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
    int teamColor; // 1 is blue, 2 is red
    boolean amplify = false;
    boolean coopertition = false;
    Timer timer = new Timer();

    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
        timer.reset();
    }

    private static Spark lightsController = new Spark(9);

    
    @Override
    public void periodic(){
        if (DriverStation.getAlliance().get().toString() == "Blue") {
            teamColor = 1;
        } else if (DriverStation.getAlliance().get().toString() == "Red") {
            teamColor = 2;
        }
    }


    public boolean isCoopertition() {
        return coopertition;
    }

    public void setCoopertition(boolean coop) {
        this.coopertition = coop;
    }

    public void off(){
        lightsController.set(0.99);
    }
    public void darkGray(){
        lightsController.set(0.97);
    }
    public void gray(){
        lightsController.set(0.95);
    }
    public void white(){
        lightsController.set(0.93);
    }
    public void violet(){
        lightsController.set(0.91);
    }
    public void blueViolet(){
        lightsController.set(0.89);
    }
    public void blue(){
        lightsController.set(0.87);
    }
    public void darkBlue(){
        lightsController.set(0.85);
    }
    public void skyBlue(){
        lightsController.set(0.83);
    }
    public void aqua(){
        lightsController.set(0.81);
    }
    public void blueGreen(){
        lightsController.set(0.79);
    }
    public void green(){
        lightsController.set(0.77);
    }
    public void darkGreen(){
        lightsController.set(0.75);
    }
    public void lime(){
        lightsController.set(0.73);
    }
    public void lawnGreen(){
        lightsController.set(0.71);
    }
    public void yellow(){
        lightsController.set(0.69);
    }
    public void gold(){
        lightsController.set(0.67);
    }
    public void orange(){
        lightsController.set(0.65);
    }
    public void redOrange(){
        lightsController.set(0.63);
    }
    public void red(){
        lightsController.set(0.61);
    }
    public void darkRed(){
        lightsController.set(0.59);
    }
    public void hotPink(){
        lightsController.set(0.57);
    }
    
    public Command amplificationCommand (){
        return Commands.repeatingSequence(
            blueCommand(),
            Commands.waitSeconds(0.2),

            redCommand(),
            Commands.waitSeconds(0.2)
        ).withTimeout(5);
    }

    public Command rainbowLights() {
        return Commands.repeatingSequence(
                greenCommand(),
                Commands.waitSeconds(0.2),
                aquaCommand(),
                Commands.waitSeconds(0.2),
                redCommand(),
                Commands.waitSeconds(0.2),
                blueCommand(),
                Commands.waitSeconds(0.2),
                whiteCommand(),
                Commands.waitSeconds(0.2),
                yellowCommand(),
                Commands.waitSeconds(0.2),
                violetCommand(),
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

    public Command offCommand(){
        return Commands.runOnce(()-> off(), this);
        }
    public Command darkGrayCommand(){
        return Commands.runOnce(()-> darkGray(), this);
    }
    public Command grayCommand(){
        return Commands.runOnce(()-> gray(), this);
    }
    public Command whiteCommand(){
        return Commands.runOnce(()-> white(), this);
    }
    public Command violetCommand(){
        return Commands.runOnce(()-> violet(), this);
    }
    public Command blueVioletCommand(){
        return Commands.runOnce(()-> blueViolet(), this);
    }
    public Command blueCommand(){
        return Commands.runOnce(()-> blue(), this);
    }
    public Command darkBlueCommand(){
        return Commands.runOnce(()-> darkBlue(), this);
    }
    public Command skyBlueCommand(){
        return Commands.runOnce(()-> skyBlue(), this);
    }
    public Command aquaCommand(){
        return Commands.runOnce(()-> aqua(), this);
    }
    public Command blueGreenCommand(){
        return Commands.runOnce(()-> blueGreen(), this);
    }
    public Command greenCommand(){
        return Commands.runOnce(()-> green(), this);
    }
    public Command darkGreenCommand(){
        return Commands.runOnce(()-> darkGreen(), this);
    }
    public Command limeCommand(){
        return Commands.runOnce(()-> lime(), this);
    }
    public Command lawnGreeCommand(){
        return Commands.runOnce(()-> lawnGreen(), this);
    }
    public Command yellowCommand(){
        return Commands.runOnce(()-> yellow(), this);
    }
    public Command goldCommand(){
        return Commands.runOnce(()-> gold(), this);
    }
    public Command orangeCommand(){
        return Commands.runOnce(()-> orange(), this);
    }
    public Command redOrangeCommand(){
        return Commands.runOnce(()-> redOrange(), this);
    }
    public Command redCommand(){
        return Commands.runOnce(()-> red(), this);
    }
    public Command darkRedCommand(){
        return Commands.runOnce(()-> darkRed(), this);
    }
    public Command hotPinkCommand(){
        return Commands.runOnce(()-> hotPink(), this);
    }
}