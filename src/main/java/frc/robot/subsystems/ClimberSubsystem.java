// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {

  private final SparkFlex m_leftClimberMotor;
  private final SparkFlex m_rightClimberMotor;

  public ClimberSubsystem() {

    m_leftClimberMotor = new SparkFlex(ClimberConstants.kClimberLeftCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig leftClimberConfig = new SparkFlexConfig();
    leftClimberConfig.idleMode(IdleMode.kBrake);

    m_leftClimberMotor.configure(
      leftClimberConfig,
      com.revrobotics.ResetMode.kResetSafeParameters,
      com.revrobotics.PersistMode.kPersistParameters);


    m_rightClimberMotor = new SparkFlex(ClimberConstants.kClimberRightCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig rightClimberConfig = new SparkFlexConfig();
    rightClimberConfig.idleMode(IdleMode.kBrake);

    m_rightClimberMotor.configure(
      rightClimberConfig,
      com.revrobotics.ResetMode.kResetSafeParameters,
      com.revrobotics.PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
