// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberSetpoints;


public class ClimberSubsystem extends SubsystemBase {

  private final SparkFlex m_climberMotor;
  // private final SparkFlex m_leftClimberMotor;
  // private final SparkFlex m_rightClimberMotor;
  private SparkClosedLoopController m_climberPidController;
  private SparkAbsoluteEncoder m_climberAbsEncoder;

  public ClimberSubsystem() {

    m_climberMotor = new SparkFlex(ClimberConstants.kClimberCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig climberConfig = new SparkFlexConfig();
    climberConfig.idleMode(IdleMode.kBrake);
    climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    climberConfig.closedLoop.pid(0.1, 0.0, 0.0);
    climberConfig.closedLoop.outputRange(0, 1);

    climberConfig.softLimit.forwardSoftLimit(0.8);
    climberConfig.softLimit.forwardSoftLimitEnabled(true);

    climberConfig.softLimit.reverseSoftLimit(0.0);
    climberConfig.softLimit.reverseSoftLimitEnabled(true);
    //added via ChatGPT so not sure about this

    // Optional: limit output for safety

    m_climberMotor.configure(
      climberConfig,
      com.revrobotics.ResetMode.kResetSafeParameters,
      com.revrobotics.PersistMode.kPersistParameters);


    // m_rightClimberMotor = new SparkFlex(ClimberConstants.kClimberRightCanId, SparkFlex.MotorType.kBrushless);
    // SparkFlexConfig rightClimberConfig = new SparkFlexConfig();
    // rightClimberConfig.idleMode(IdleMode.kBrake);

    // m_rightClimberMotor.configure(
    //   rightClimberConfig,
    //   com.revrobotics.ResetMode.kResetSafeParameters,
    //   com.revrobotics.PersistMode.kPersistParameters
    // );


    m_climberPidController = m_climberMotor.getClosedLoopController();
    m_climberAbsEncoder = m_climberMotor.getAbsoluteEncoder();


// setPosition(ClimberSetpoints.kAway);

  }
  // if (setpoint == ClimberSetpoints.kAway) {
  //   // STOW → always go DOWN
  //   m_climberMotor.set(-ClimberConstants.kClimberSpeed);
  // } else if (setpoint == ClimberSetpoints.kLockout) {
  //   // RAISE → always go UP
  //   m_climberMotor.set(+ClimberConstants.kClimberSpeed);
  // }
    // public void setPosition(double position) {
    //   m_climberPidController.setSetpoint(position, ControlType.kPosition);
    // }

public void setPosition(double setpoint) {
    m_climberPidController.setSetpoint(setpoint, ControlType.kPosition);
}




  // public void climberUp() {
  //   setPosition(ClimberSetpoints.kRaise);
  // }
  public void climberPull() {
    setPosition(ClimberSetpoints.kLockout);
  }
  public void climberAway() {
    setPosition(ClimberSetpoints.kAway);
  }
  public void climberStop(double speed) {
    m_climberMotor.set(speed);
  }

  public boolean isClimberAtLockout() {
    return m_climberPidController.isAtSetpoint();
}
public boolean isClimberAtStow() {
    return m_climberPidController.isAtSetpoint();
}

  public Command Lockout(){
    return this.startEnd(
      this::climberPull,
      () -> { this.m_climberMotor.stopMotor(); }
    ).until(this::isClimberAtLockout);
  }
  public Command climberStow(){
    return this.startEnd(
      this::climberAway,
      () -> { this.m_climberMotor.stopMotor(); }
    ).until(this::isClimberAtStow);
  }
  public Command climberStop() {
    return this.startEnd(
        () -> m_climberMotor.stopMotor(),
        () -> m_climberMotor.stopMotor()
    );
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber Position", m_climberAbsEncoder.getPosition());

  }
}

