// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SwerveDriveWithController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionalSubsystem;
import frc.robot.subsystems.SwerveDrivebase;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
    private final Joystick topButtonBoard = new Joystick(Constants.OperatorConstants.kTopButtonBoard);
    private final Joystick bottomButtonBoard = new Joystick(Constants.OperatorConstants.kBottomButtonBoard);
    private final SwerveDrivebase swerveDrivebase = new SwerveDrivebase(Constants.SwerveConstants.swerveModuleArray);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    TalonFX intakePivot = new TalonFX(Constants.ElectronicsPorts.intakePivot);

    JoystickButton a = new JoystickButton(controller, 1);
    JoystickButton b = new JoystickButton(controller, 2);
    JoystickButton leftBumper = new JoystickButton(controller, 5);
    JoystickButton rightBumper = new JoystickButton(controller, 6);

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

    final PositionalSubsystem intakeSegment = new PositionalSubsystem(
            8,
            0,
            intakePivot,
            new PIDController(1, 0, 0),
            -20,
            180,
            3,
            false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        SmartDashboard.putData("Intake Segment", intakeSegment);
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
        swerveDrivebase.setDefaultCommand(new SwerveDriveWithController(swerveDrivebase, controller));
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
        a.whileTrue(new IntakeCommand(intakeSubsystem, true));
        b.whileTrue(new IntakeCommand(intakeSubsystem, false));

        redOne.whileTrue(intakeSegment.smoothMoveToAngle(0));


        leftBumper.whileTrue(intakeSegment.smoothMoveToAngle(0));
        rightBumper.whileTrue(intakeSegment.smoothMoveToAngle(120));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        return null; // TODO
    }
}
