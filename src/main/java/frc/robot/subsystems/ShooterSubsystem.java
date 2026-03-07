package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_shooterLeftMotor;
  private final SparkFlex m_shooterRightMotor;
  private final SparkFlex m_agitatorMotor;
  // private final SparkFlex m_thumperMotor;

  private final SparkClosedLoopController m_shooterPID;

  public ShooterSubsystem() {
    m_shooterLeftMotor = new SparkFlex(ShooterConstants.kShooterLeftCanId, SparkFlex.MotorType.kBrushless);
  SparkFlexConfig shooterLeftConfig = new SparkFlexConfig();
  shooterLeftConfig.idleMode(IdleMode.kCoast);
  shooterLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  shooterLeftConfig.closedLoop.pid(0.00001, 0, 0);
  shooterLeftConfig.closedLoop.feedForward.sv(0.35, 0.00175);

  m_shooterLeftMotor.configure(
      shooterLeftConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
      );

  m_shooterPID = m_shooterLeftMotor.getClosedLoopController();



    m_shooterRightMotor = new SparkFlex(ShooterConstants.kShooterRightCanId, SparkFlex.MotorType.kBrushless);
  SparkFlexConfig shooterRightConfig = new SparkFlexConfig();
  shooterRightConfig.idleMode(IdleMode.kCoast);
  shooterRightConfig.follow(m_shooterLeftMotor, true);
  m_shooterRightMotor.configure(
      shooterRightConfig,
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

//       m_thumperMotor = new SparkFlex(ShooterConstants.kThumperCanId, SparkFlex.MotorType.kBrushless);

//     SparkFlexConfig thumperConfig = new SparkFlexConfig();
//   thumperConfig.idleMode(IdleMode.kCoast);
//   thumperConfig.follow(m_agitatorMotor, false);
// //We were thinking of adding setpoints to this to further agitate our hopper, but for now it just runs in a circle.

//   m_thumperMotor.configure(
//       thumperConfig,
//       ResetMode.kResetSafeParameters,
//       PersistMode.kPersistParameters
//       );
  }

  public void setShooterSpeed(double setpoint) {
    m_shooterPID.setSetpoint(setpoint, ControlType.kVelocity);
  }
  public void setAgitatorSpeed(double speed) {
    m_agitatorMotor.set(speed);
  }
  // public void setThumperSpeed(double speed) {
  //   m_thumperMotor.set(speed);
  // }


  public Command runShooter() {
    return this.startEnd(
      () -> this.setShooterSpeed(ShooterConstants.kShooterLeftSpeed),
      m_shooterLeftMotor::stopMotor);
  }
  public Command runAgitator() {
    return this.startEnd(
      () -> this.setAgitatorSpeed(ShooterConstants.kAgitatorSpeed),
      m_agitatorMotor::stopMotor);
  }


  public Command runShooterAndAgitate() {
    return Commands.sequence(
      this.run(() -> {this.setShooterSpeed(ShooterConstants.kShooterLeftSpeed);}).withTimeout(0.5),
      this.run(() -> {
        this.setShooterSpeed(ShooterConstants.kShooterLeftSpeed);
        this.setAgitatorSpeed(ShooterConstants.kAgitatorSpeed);
      })
    );
  }

  public Command stopShooterAndAgitator() {
    return this.run(() -> {
        this.m_shooterLeftMotor.stopMotor();
        this.m_agitatorMotor.stopMotor();});
  }


    public Command reverseShooterAndAgitate() {
    return this.startEnd(
      () -> {
        this.setShooterSpeed(ShooterConstants.kReverseShooterSpeed);
        this.setAgitatorSpeed(ShooterConstants.kReverseAgitatorSpeed);

      },
      () -> {
        this.m_shooterLeftMotor.stopMotor();
        this.m_agitatorMotor.stopMotor();
      }
    );
  }



  public Command reverseShooter() {
    return this.startEnd(() -> this.setShooterSpeed(ShooterConstants.kReverseShooterSpeed),
    () -> this.setShooterSpeed(0));
  }
  public Command reverseAgitator() {
    return this.startEnd(() -> this.setAgitatorSpeed(ShooterConstants.kReverseAgitatorSpeed),
    () -> this.setAgitatorSpeed(0));
  }


    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooterSpeed", m_shooterLeftMotor.getEncoder().getVelocity());

  }

}


