package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

// import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSystem {

  public ShooterSystem() {
    m_shooterMotor = new SparkFlex(18, SparkFlex.MotorType.kBrushless);

    m_shooterMotor.setIdleMode(SparkFlex.IdleMode.kCoast);
  }
SparkFlexConfig config = new SparkFlexConfig();

config.idleMode(SparkFlex.IdleMode.kCoast);



  public void setShooterSpeed(double speed) {
    m_shooterMotor.set(speed);
  }


private final SparkFlex m_shooterMotor = new SparkFlex(18, SparkFlex.MotorType.kBrushless);


}
