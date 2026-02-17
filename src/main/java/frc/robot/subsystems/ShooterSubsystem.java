package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_shooterMotor;
  private final SparkFlex m_agitatorMotor;

  public ShooterSubsystem() {
    m_shooterMotor = new SparkFlex(ShooterConstants.kShooterCanId, SparkFlex.MotorType.kBrushless);
  SparkFlexConfig shooterConfig = new SparkFlexConfig();
  shooterConfig.idleMode(IdleMode.kCoast);

  m_shooterMotor.configure(
      shooterConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
      );
    

      m_agitatorMotor = new SparkFlex(ShooterConstants.kAgitatorCanId, SparkFlex.MotorType.kBrushless);

    SparkFlexConfig agitatorConfig = new SparkFlexConfig();
  agitatorConfig.idleMode(IdleMode.kCoast);

  m_agitatorMotor.configure(
      agitatorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
      
      );

  }

  public void setShooterSpeed(double speed) {
    m_shooterMotor.set(speed);
  }

    public void setAgitatorSpeed(double speed) {
    m_agitatorMotor.set(speed);
  }


  
  public Command runShooter() {
    return this.run(() -> this.setShooterSpeed(ShooterConstants.kShooterSpeed));
  }

  public Command runAgitator() {
    return this.run(() -> this.setShooterSpeed(ShooterConstants.kAgitatorSpeed));
  }







    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}


