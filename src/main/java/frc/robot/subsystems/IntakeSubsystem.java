// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeSetpoints;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;


public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex m_intakeMotor;
  private final SparkFlex m_flipperMotor;

  private SparkClosedLoopController m_flipperPidController;
  private SparkAbsoluteEncoder m_flipperAbsEncoder;
  // Example: assuming a simple PIDController

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.inverted(true);

    m_intakeMotor.configure(
      intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_flipperMotor = new SparkFlex(IntakeConstants.kFlipperCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig flipperConfig = new SparkFlexConfig();

    flipperConfig.idleMode(IdleMode.kBrake);
    flipperConfig.inverted(true);
    flipperConfig.smartCurrentLimit(40);

    // PID values (starting values)
    flipperConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    flipperConfig.closedLoop.pid(3.0, 0.0, 0.0);
    flipperConfig.closedLoop.allowedClosedLoopError(0.035, ClosedLoopSlot.kSlot0);
    flipperConfig.smartCurrentLimit(30);
    // flipperConfig.closedLoop.allowedClosedLoopError(0.002, ClosedLoopSlot.kSlot0);

    // Optional: limit output for safety
    flipperConfig.closedLoop.outputRange(-1, 1);

    m_flipperMotor.configure(
      flipperConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_flipperPidController = m_flipperMotor.getClosedLoopController();
    m_flipperAbsEncoder = m_flipperMotor.getAbsoluteEncoder();

    SmartDashboard.putNumber("Flipper Angle", m_flipperAbsEncoder.getPosition());
    SmartDashboard.putBoolean("Flipper At Setpoint", atStowPosition() || atDroppedPosition());

    setPosition(IntakeSetpoints.kStow);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public Command runIntake() {
    return this.startEnd(
      () -> this.setIntakeSpeed(IntakeConstants.kIntakeSpeed),
      () -> this.setIntakeSpeed(0));
  }

  public Command runIntakeWhileStowed() {
    return this.startEnd(
      () -> {
        this.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        setPosition(IntakeSetpoints.kStow);
      },
      () -> {
        this.setIntakeSpeed(0);
        this.m_flipperMotor.stopMotor();
      }
    );
  }

  public Command runIntakeAndDropFlipper() {
    return this.startEnd(
      () -> {
        this.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        this.feedFlipper();
      },
      () -> {
        this.setIntakeSpeed(0);
        this.m_flipperMotor.stopMotor();
      }
    );
  }

  public Command reverseIntake() {
    return this.startEnd(
    () -> this.setIntakeSpeed(IntakeConstants.kReverseIntakeSpeed),
    () -> this.setIntakeSpeed(0));
  }

  public Command dropFlipper() {
    return this.startEnd(
      this::feedFlipper,
      () -> { this.m_flipperMotor.stopMotor(); }
    ).until(this::atDroppedPosition);
  }

  public void setPosition(double position) {
    m_flipperPidController.setSetpoint(position, ControlType.kPosition);
  }

  public void feedFlipper() {
    setPosition(IntakeSetpoints.kFeeding);
  }

  public boolean atStowPosition() {
    return Math.abs(m_flipperAbsEncoder.getPosition() - IntakeSetpoints.kStow) <= 0.035;
  }
  public boolean atDroppedPosition() {
    return Math.abs(m_flipperAbsEncoder.getPosition() - IntakeSetpoints.kFeeding) <= 0.035;
  }

  public Command stowFlipper() {
    return this.run(() -> setPosition(IntakeSetpoints.kStow)).until(this::atStowPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper Angle", m_flipperAbsEncoder.getPosition());
    SmartDashboard.putNumber("Flipper Commanded Angle", m_flipperPidController.getSetpoint());
    SmartDashboard.putBoolean("Flipper At Setpoint", atStowPosition() || atDroppedPosition());
    SmartDashboard.putNumber("Flipper Current Amps", m_flipperMotor.getOutputCurrent());
  }
}
