// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex m_intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
  

      m_intakeMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
        );
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public Command runIntake() {
    return this.run(() -> this.setIntakeSpeed(IntakeConstants.kIntakeSpeed));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
