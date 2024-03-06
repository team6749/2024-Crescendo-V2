// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveWithController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindLTV;
import com.pathplanner.lib.util.GeometryUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    //Initializing any USB plugins for robot control, in our case: 1 xbox controller(defined using XBoxController or CommandXboxController class), 
    //can be substituted with PS5 Controller by using CommandPS5Controller class and two button boards(defined as joysticks)
    private final XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
    private final Joystick topButtonBoard = new Joystick(Constants.OperatorConstants.kTopButtonBoard);
    private final Joystick bottomButtonBoard = new Joystick(Constants.OperatorConstants.kBottomButtonBoard);
    
    //Subsystems
    private final SwerveDrivebase swerveDrivebase = new SwerveDrivebase(Constants.SwerveConstants.swerveModuleArray);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final NoteDetection noteDetection = new NoteDetection();
    TalonFX intakePivot = new TalonFX(Constants.ElectronicsPorts.intakePivot);
    
    //Buttons For Driver Controller
    JoystickButton a = new JoystickButton(controller, 1);
    JoystickButton b = new JoystickButton(controller, 2);
    JoystickButton x = new JoystickButton(controller, 3);
    JoystickButton y = new JoystickButton(controller, 4);
    JoystickButton leftBumper = new JoystickButton(controller, 5);
    JoystickButton rightBumper = new JoystickButton(controller, 6);

    Trigger dpad_up = new Trigger(() -> controller.getPOV() == 180);
    Trigger dpad_left = new Trigger(() -> controller.getPOV() == 270);
    Trigger dpad_down = new Trigger(() -> controller.getPOV() == 0);
    Trigger dpad_right = new Trigger(() -> controller.getPOV() == 90);

    //Buttons For Top Button Board (red and yellow)
    JoystickButton redOne = new JoystickButton(topButtonBoard, 1);
    JoystickButton redTwo = new JoystickButton(topButtonBoard, 2);
    JoystickButton redThree = new JoystickButton(topButtonBoard, 3);
    JoystickButton redFour = new JoystickButton(topButtonBoard, 4);
    JoystickButton redFive = new JoystickButton(topButtonBoard, 5);
    JoystickButton yellowOne = new JoystickButton(topButtonBoard, 6);
    JoystickButton yellowTwo = new JoystickButton(topButtonBoard, 7);
    JoystickButton yellowThree = new JoystickButton(topButtonBoard, 8);
    JoystickButton yellowFour = new JoystickButton(topButtonBoard, 9);
    JoystickButton yellowFive = new JoystickButton(topButtonBoard, 10);

    //Buttons For Bottom Button Board (blue and green)
    JoystickButton blueOne = new JoystickButton(bottomButtonBoard, 1);
    JoystickButton blueTwo = new JoystickButton(bottomButtonBoard, 2);
    JoystickButton blueThree = new JoystickButton(bottomButtonBoard, 3);
    JoystickButton blueFour = new JoystickButton(bottomButtonBoard, 4);
    JoystickButton blueFive = new JoystickButton(bottomButtonBoard, 5);
    JoystickButton greenOne = new JoystickButton(bottomButtonBoard, 6);
    JoystickButton greenTwo = new JoystickButton(bottomButtonBoard, 7);
    JoystickButton greenThree = new JoystickButton(bottomButtonBoard, 8);
    JoystickButton greenFour = new JoystickButton(bottomButtonBoard, 9);
    JoystickButton greenFive = new JoystickButton(bottomButtonBoard, 10);

    // final PositionalSubsystem intakeSegment = new PositionalSubsystem(
    // 8,
    // 0,
    // intakePivot,
    // new PIDController(1, 0, 0),
    // -20,
    // 180,
    // 3,
    // false);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //Calling this sends any data put in a sendable builder or any other data to the shuffleboard application. 
        //Driver station should automatically open shuffleboard when opened, if it does not, search it up in windows and pick the right year version (if given multiple options)
        SmartDashboard.putData("Shooter Subsystem", shooterSubsystem);
        SmartDashboard.putData("Swerve Subsystem", swerveDrivebase);
        SmartDashboard.putData("Intake Subsystem", intakeSubsystem);
        // SmartDashboard.putData("Auto Chooser", autoChooser);
        
        //Function that actually activates the different commands
        configureBindings();


        //Acesses any built autonomous paths from PathPlanner and puts them as options in the auto builder
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Build an auto chooser. This will use Commands.none() as the default option.
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        //Adds any commands we made in the code directly to PathPlanner to be used in autonomous paths
        NamedCommands.registerCommand("Shoot Speaker", shootSpeaker());
        NamedCommands.registerCommand("Shoot Amp", shootAmp());

        swerveDrivebase
                .setDefaultCommand(new SwerveDriveWithController(swerveDrivebase, controller, blueFive, noteDetection));

        SmartDashboard.putData("Shooter Subsystem", shooterSubsystem);
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));
        // NamedCommands.registerCommand("Test Command", ampScoringAuto());
        

        //Button to intake notes from the source
        a.whileTrue(sourceIntake());

        //Button to shoot into speaker
        x.whileTrue(shootSpeaker());

        //Button to shoot into the amp
        y.whileTrue(shootAmp());

        //Button to intake notes from the ground
        dpad_down.whileTrue(groundIntake());
        //Button to intake from the source
        dpad_up.whileTrue(sourceIntake());
        
        //Buttons to shoot into speaker
        dpad_left.whileTrue(shootSpeaker());
        dpad_right.whileTrue(shootSpeaker());

        //All red buttons on the button board run the command to shoot into the speaker
        redOne.whileTrue(shootSpeaker());
        redTwo.whileTrue(shootSpeaker());
        redThree.whileTrue(shootSpeaker());
        redFour.whileTrue(shootSpeaker());
        redFive.whileTrue(shootSpeaker());

        //All yellow buttons on the button board run the command to shoot into the amp
        yellowOne.whileTrue(shootAmp());
        yellowTwo.whileTrue(shootAmp());
        yellowThree.whileTrue(shootAmp());
        yellowFour.whileTrue(shootAmp());
        yellowFive.whileTrue(shootAmp());
        
        //All blue buttons on the button board run the command to intake from the ground
        blueOne.onTrue(groundIntake());
        blueTwo.whileTrue(groundIntake());
        blueThree.whileTrue(groundIntake());
        blueFour.whileTrue(groundIntake());
        blueFive.whileTrue(groundIntake());

        //All green buttons on the button board run the command to intake from source (drop into robot)
        greenOne.whileTrue(sourceIntake());
        greenTwo.whileTrue(sourceIntake());
        greenThree.whileTrue(sourceIntake());
        greenFour.whileTrue(sourceIntake());
        greenFive.whileTrue(sourceIntake());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        System.out.println("(" + autoChooser.getSelected().getName() + ")");

        // System.out.println(PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName()));
        return autoChooser.getSelected();
        // System.out.println(autoChooser.getSelected().toString());
        // return new PathPlannerAuto(autoChooser.getSelected().toString());
        // return new PathPlannerAuto("0 Speaker Leave Top");
    }

    //StartEnd Commands are very useful, they take 3 inputs: the functions to run when command starts, functions to run when command ends
    // and the subsystem used, and the subsystems any functions you used are in. They also take an additional optional timeout function
    /*public Command exampleStartEndCommand(){
        return Commands.startEnd(
            ()-> {
                functions to run when command is initialized
                    e.g.: run motors
            },
            ()->{
                functions to run when command ends
                    e.g.: turn off motors
            },
            subsystem used
            ).withTimeout(amount of time to run the command); 
    }
     */    

    public Command shootSpeaker() {
        return Commands.startEnd(
                () -> {
                    shooterSubsystem.shoot(9);
                    intakeSubsystem.indexNote(false, true);
                },
                () -> {
                    shooterSubsystem.shoot(0);
                    intakeSubsystem.stopIndexer();
                },
                shooterSubsystem, intakeSubsystem
                ).withTimeout(1);
    }

    public Command shootAmp() {
        return Commands.startEnd(
                () -> {
                    shooterSubsystem.shoot(2);
                    intakeSubsystem.indexNote(false, true);
                },
                () -> {
                    shooterSubsystem.shoot(0);
                    intakeSubsystem.stopIndexer();
                },
                shooterSubsystem).withTimeout(1);
    }

    public Command sourceIntake() {
        return Commands.startEnd(
                () -> {
                    shooterSubsystem.shooterIntake();
                    intakeSubsystem.indexNote(true, false);
                },
                () -> {
                    shooterSubsystem.shoot(0);
                    intakeSubsystem.stopIndexer();
                }, shooterSubsystem).withTimeout(1);
    }

    public Command groundIntake(){
        return Commands.startEnd(
                ()->{
                    intakeSubsystem.intake(true, 1);
                    intakeSubsystem.indexNote(false, false);
                },
                ()->{
                    intakeSubsystem.stopIndexer();
                    intakeSubsystem.stopIntake();
                }, intakeSubsystem).withTimeout(10);
    }


    public Command ampScoringAuto(){
        return Commands.print("Hello");
    }

}
