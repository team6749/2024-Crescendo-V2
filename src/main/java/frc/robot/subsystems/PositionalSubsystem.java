// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PositionalSubsystem extends PIDSubsystem {
	public MotorController motor;
	public DutyCycleEncoder encoder;

	private double minRange;
	private double maxRange;
	private double maxOutput;
	private double calibrationOffset;
	private boolean inverted;

	public PositionalSubsystem(int encoderPort, double calibrationOffset, MotorController motor,
			PIDController pidcontroller,
			double minRange, double maxRange, double maxOutput, boolean inverted) {
		super(pidcontroller);
		this.motor = motor; // MotorController class converts thee passed in motor into the correct type!
		encoder = new DutyCycleEncoder(encoderPort);

		this.minRange = minRange;
		this.maxRange = maxRange;
		this.maxOutput = maxOutput;
		this.inverted = inverted;
		this.calibrationOffset = calibrationOffset;

		// degrees
		pidcontroller.setTolerance(1);

		// By default do not move
		disable();
	}

	double velocity = 0;
	double lastMeasurement = 0;
	double lastTime = 0;

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (getSetpoint() < minRange) {
			setSetpoint(minRange);
		}
		if (getSetpoint() > maxRange) {
			setSetpoint(maxRange);
		}

		double time = Timer.getFPGATimestamp();
		double measurement = getMeasurement();
		velocity = (measurement - lastMeasurement) * (1d/(time - lastTime)); // 0.02 delta time for loop
		lastMeasurement = measurement;
		lastTime = time;

		super.periodic();
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		// Use the output here
		if (output > maxOutput) {
			output = maxOutput;
		}
		if (output < -maxOutput) {
			output = -maxOutput;
		}

		if (getMeasurement() < minRange) {
			motor.setVoltage(0);
			motor.stopMotor();
			return;
		}
		if (getMeasurement() > maxRange) {
			motor.setVoltage(0);
			motor.stopMotor();
			return;
		}
		motor.setVoltage(-output);
	}

	@Override
	public double getMeasurement() { // in degrees
		return (inverted) ? (-((encoder.getAbsolutePosition() * 360d) + calibrationOffset))
				: ((encoder.getAbsolutePosition() * 360d) + calibrationOffset);
	}

	public double getAngle() {
		return getMeasurement();
	}

	public double getPower() {
		return motor.get();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("PositionalSubsystem");
		builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
		builder.addDoubleProperty("Measurement", this::getMeasurement, null); // not working
		builder.addBooleanProperty("IsAtSetpoint", this::atSetpoint, null);
		builder.setSafeState(this::disable);
		builder.addDoubleProperty("Arm motor power", this::getPower, null);
		builder.addDoubleProperty("velocity", () -> velocity, null);
		builder.addBooleanProperty("encoder connected", () -> encoder.isConnected(), null);
		SmartDashboard.putData(getName() + " pid", this.getController());
	}

	// we cannot be at the setpoint
	public boolean atSetpoint () {
		return m_controller.atSetpoint();
		//return m_enabled && Math.abs(m_controller.getSetpoint() - getMeasurement()) < m_controller.getPositionTolerance();
	}

	public Command smoothMoveToAngle(double targetAngle) {
		// if(isIntakeDeployed) {
			return new TrapezoidProfileCommand(
					// The motion profile to be executed
					new TrapezoidProfile(
							// The motion profile constraints
							new TrapezoidProfile.Constraints(3, 2)),
					state -> {
						// Use current trajectory state here
						setSetpoint(state.position);
					},
					// Goal state
					() -> new TrapezoidProfile.State(targetAngle, 0),
					// Current state
					// TrapezoidProfile.State::new,
					() -> new TrapezoidProfile.State(getAngle(), 0),
					this).beforeStarting(this::enable).until(() -> Math.abs(targetAngle - getAngle()) < m_controller.getPositionTolerance() );
		// }else{
		// return new Command(){};
		// }
	}
}