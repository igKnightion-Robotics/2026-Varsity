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
import frc.robot.Constants.IntakeSetpoints;



public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex m_intakeMotor;
    private final SparkFlex m_flipperMotor;
    
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeCanId, SparkFlex.MotorType.kBrushless);
  SparkFlexConfig intakeConfig = new SparkFlexConfig();
  intakeConfig.idleMode(IdleMode.kCoast);

  m_intakeMotor.configure(
      intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

//flipper motor configuring

  m_flipperMotor = new SparkFlex(IntakeConstants.kFlipperCanId, SparkFlex.MotorType.kBrushless);
  SparkFlexConfig flipperConfig = new SparkFlexConfig();
  flipperConfig.idleMode(IdleMode.kBrake);

  m_flipperMotor.configure(
      flipperConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
}




  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public Command runIntake() {
    return this.run(() -> this.setIntakeSpeed(IntakeConstants.kIntakeSpeed));
  }


  //need to add a stow method, only goes to feeded setpoint right now, but we need to be able to stow it as well
  public Command runFlipper() {
    return this.run(() -> this.setIntakeSpeed(IntakeSetpoints.kFeeding));
  } 
  
  public Command stowFlipper() {
    return this.run(() -> this.setIntakeSpeed(IntakeSetpoints.kStow));
  }

  // public Command setSetpointcommand() {

  // }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
