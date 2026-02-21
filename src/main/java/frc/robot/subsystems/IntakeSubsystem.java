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


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeSetpoints;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;




public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex m_intakeMotor;
    private final SparkFlex m_flipperMotor;

    private SparkClosedLoopController m_pidController;
    private SparkAbsoluteEncoder m_absEncoder;

    
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

// m_flipperMotor.setSoftLimits(-10, 120); //min/max degrees

  flipperConfig.idleMode(IdleMode.kBrake);



// PID values (starting values)
flipperConfig.closedLoop.pid(0.01, 0.0, 0.0);

// Optional: limit output for safety
flipperConfig.closedLoop.outputRange(-0.5, 0.5);


  m_flipperMotor.configure(
      flipperConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);



m_pidController = m_flipperMotor.getClosedLoopController();
m_absEncoder = m_flipperMotor.getAbsoluteEncoder();

}



  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }



  public Command runIntake() {
    return this.run(() -> this.setIntakeSpeed(IntakeConstants.kFlipperSpeed));
  }






  //i don't think this is right, but who knows?
double gearRatio = 25.0; // 25:1 gearbox

public void setPosition(double positionDegrees) {

   double motorDegrees = positionDegrees * gearRatio;

    m_pidController.setReference(motorDegrees, ControlType.kPosition);
}


  public Command feedFlipper() {
    return this.runOnce(() -> setPosition(IntakeSetpoints.kFeeding));
}

public Command stowFlipper() {
    return this.runOnce(() -> setPosition(IntakeSetpoints.kStow));
}




  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
