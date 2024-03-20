// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveWithController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrivebase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    // Initializing any USB plugins for robot control, in our case: 1 xbox
    // controller(defined using XBoxController or CommandXboxController class),
    // can be substituted with PS5 Controller by using CommandPS5Controller class
    // and two button boards(defined as joysticks)
    private final XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
    private final Joystick topButtonBoard = new Joystick(Constants.OperatorConstants.kTopButtonBoard);
    private final Joystick bottomButtonBoard = new Joystick(Constants.OperatorConstants.kBottomButtonBoard);

    // Subsystems
    private final SwerveDrivebase swerveDrivebase;
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Buttons For Driver Controller
    JoystickButton a = new JoystickButton(controller, 1);
    JoystickButton b = new JoystickButton(controller, 2);
    JoystickButton x = new JoystickButton(controller, 3);
    JoystickButton y = new JoystickButton(controller, 4);
    JoystickButton leftBumper = new JoystickButton(controller, 5);
    JoystickButton rightBumper = new JoystickButton(controller, 6);

    Trigger left_trigger = new Trigger(() -> controller.getLeftTriggerAxis() > 0.1);
    Trigger right_trigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.1);

    Trigger dpad_up = new Trigger(() -> controller.getPOV() == 0);
    Trigger dpad_left = new Trigger(() -> controller.getPOV() == 270);
    Trigger dpad_down = new Trigger(() -> controller.getPOV() == 180);
    Trigger dpad_right = new Trigger(() -> controller.getPOV() == 90);

    JoystickButton start_button = new JoystickButton(controller, 8);
    JoystickButton back_button = new JoystickButton(controller, 7);

    // Buttons For Top Button Board (red and yellow)
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

    // Buttons For Bottom Button Board (blue and green)
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
        List<PointOfInterest> pointsOfInterest = new ArrayList<>();

        pointsOfInterest.add(0, new PointOfInterest("Blue Amp", new Translation2d(1.83, 7.75), Rotation2d.fromDegrees(-90), 5, 0.05));
        // THIS WAS TESTED AS PRETTY CLOSE BUT IT WAS WITH A LOW BATTERY. we can use this as a reference for the other ones
        pointsOfInterest.add(1, new PointOfInterest("Blue Stage up", new Translation2d(4.15, 5.26), Rotation2d.fromDegrees(119.5), 1, 0.1));
        pointsOfInterest.add(2, new PointOfInterest("Blue Stage down", new Translation2d(4.05, 2.75), Rotation2d.fromDegrees(-117.98), 1, 0.1));
        pointsOfInterest.add(3, new PointOfInterest("Blue Stage middle", new Translation2d(6.31, 4.06), Rotation2d.fromDegrees(0), 1, 0.1));

        pointsOfInterest.add(4, new PointOfInterest("Red Amp", new Translation2d(14.65, 7.75), Rotation2d.fromDegrees(-90),  5, 0.05));
        pointsOfInterest.add(5, new PointOfInterest("Red Stage up", new Translation2d(12.47, 5.21), Rotation2d.fromDegrees(58.74), 1, 0.1));
        pointsOfInterest.add(6, new PointOfInterest("Red Stage down", new Translation2d(12.45, 2.95), Rotation2d.fromDegrees(-58.5), 1, 0.1));
        pointsOfInterest.add(7, new PointOfInterest("Red Stage middle", new Translation2d(10.24, 3.99), Rotation2d.fromDegrees(180), 1, 0.1));
        pointsOfInterest.add(8, new PointOfInterest("TEST ZERO", new Translation2d(0, 0), Rotation2d.fromDegrees(0), 5, 0.05));
        
        swerveDrivebase = new SwerveDrivebase(Constants.SwerveConstants.swerveModuleArray, pointsOfInterest);
        
        // Calling this sends any data put in a sendable builder or any other data to
        // the shuffleboard application.
        // Driver station should automatically open shuffleboard when opened, if it does
        // not, search it up in windows and pick the right year version (if given
        // multiple options)
        SmartDashboard.putData("Shooter Subsystem", shooterSubsystem);
        SmartDashboard.putData("Swerve Subsystem", swerveDrivebase);
        SmartDashboard.putData("Intake Subsystem", intakeSubsystem);
        SmartDashboard.putData("Climber subsystem", climberSubsystem);
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        // Adds any commands we made in the code directly to PathPlanner to be used in
        // autonomous paths
        NamedCommands.registerCommand("Shoot Speaker", shootSpeaker());
        NamedCommands.registerCommand("Shoot Amp", shootAmp());
        NamedCommands.registerCommand("Shoot Trap", shootTrap());
        NamedCommands.registerCommand("Intake", intakeSubsystem.groundIntake());

        // Acesses any built autonomous paths from PathPlanner and puts them as options
        // in the auto builder
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
        // Function that actually activates the different commands
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

        // Default command, will constantly call everything in the "execute" section of
        // the command
        swerveDrivebase.setDefaultCommand(new SwerveDriveWithController(swerveDrivebase, controller));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.groundIntake());

        //unused controller inputs
        // a.whileTrue()
        // dpad_left.whileTrue()
        // dpad_right.whileTrue()
        // dpad_up.whileTrue()
        // dpad_down.whileTrue()

        //Button to shoot into the trap
        b.onTrue(shootTrap());

        // Button to shoot into speaker
        x.onTrue(shootSpeaker());

        // Button to shoot into the amp
        y.onTrue(shootAmp());

        a.whileTrue(swerveDrivebase.badJankAlignWithPoint());

        leftBumper.onTrue(swerveDrivebase.driveModeCommand());

        back_button.onTrue(swerveDrivebase.resetOdometryCommand());
        start_button.whileTrue(driveForward());

        left_trigger.whileTrue(climberSubsystem.raiseClimber());
        right_trigger.whileTrue(climberSubsystem.lowerClimber());


        redFour.onTrue(shootSpeaker());
        redFive.whileTrue(climberSubsystem.raiseClimber());

        yellowFour.onTrue(shootAmp());
        yellowFive.whileTrue(climberSubsystem.lowerClimber());

        //operator buttons
        // blueOne.whileTrue(swerveDrivebase.resetOdometryCommand());
        // blueTwo.whileTrue(driveForward());
        // greenFive.onTrue(swerveDrivebase.driveModeCommand());


        //unused buttons
        //redOne
        //redTwo
        //redThree
        //yellowOne
        //yellowTwo
        //yellowThree
        //blueThree
        //blueFour
        //blueFive
        //greenOne
        //greenTwo
        //greenThree
        //greenFour
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Gets the selected data(autonomous path) from shuffleboard that the user
        // chooses

        return autoChooser.getSelected();
    }

    public Command shootSpeaker() {
        return Commands.startEnd(
                () -> {
                    System.out.println("started shoot command");
                    shooterSubsystem.shoot(9, 1, 1);
                    intakeSubsystem.indexNote(10);
                },
                () -> {
                    System.out.println("ended shoot command");
                    shooterSubsystem.shoot(0, 1, 1);
                    intakeSubsystem.stopIndexer();
                },
                shooterSubsystem, intakeSubsystem).withTimeout(0.5);
    }

    public Command shootAmp() {
        return Commands.startEnd(
                () -> {
                    shooterSubsystem.shoot(3, 0.3, 1);
                    intakeSubsystem.indexNote(8);
                },
                () -> {
                    shooterSubsystem.shoot(0, 1, 1);
                    intakeSubsystem.stopIndexer();
                }, shooterSubsystem, intakeSubsystem).withTimeout(1);
    }


    public Command driveForward() {
        return Commands.runEnd(
                () -> {
                    swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0.2, 0, 0));
                },
                () -> {
                    swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0, 0, 0));
                }, swerveDrivebase);
    }

    public Command shootTrap() {
        return Commands.startEnd(
                () -> {
                    intakeSubsystem.indexNote(8);
                    //shooterSubsystem.shoot(shooterSubsystem.getVoltage(), shooterSubsystem.getTopShooterMaxModifier(), shooterSubsystem.getBottomShooterMaxModifier());
                    shooterSubsystem.shoot(8, 1, 0.75);
                },
                () -> {
                    intakeSubsystem.stopIndexer();
                    shooterSubsystem.shoot(0, 1, 1);
                }, intakeSubsystem, shooterSubsystem).withTimeout(2);

    }

}
