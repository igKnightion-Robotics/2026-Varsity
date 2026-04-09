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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FlywheelLookup;


public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_shooterLeftMotor;
  private final SparkFlex m_shooterRightMotor;
  private final SparkFlex m_AgitatorMotor;
  private final SparkFlex m_leftRollerMotor;
  private final SparkFlex m_rightRollerMotor;

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



    m_AgitatorMotor = new SparkFlex(ShooterConstants.kAgitatorCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig agitatorConfig = new SparkFlexConfig();
    agitatorConfig.idleMode(IdleMode.kCoast);

    m_AgitatorMotor.configure(
      agitatorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
      );

    m_leftRollerMotor = new SparkFlex(ShooterConstants.kLeftRollerCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig leftRollerConfig = new SparkFlexConfig();
    leftRollerConfig.idleMode(IdleMode.kCoast);
    leftRollerConfig.smartCurrentLimit(40);

    m_leftRollerMotor.configure(
      leftRollerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
      );

    m_rightRollerMotor = new SparkFlex(ShooterConstants.kRightRollerCanId, SparkFlex.MotorType.kBrushless);
    SparkFlexConfig rightRollerConfig = new SparkFlexConfig();
    rightRollerConfig.idleMode(IdleMode.kCoast);
    rightRollerConfig.smartCurrentLimit(40);
    rightRollerConfig.follow(m_leftRollerMotor, true);

    m_rightRollerMotor.configure(
        rightRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
      );
  }

  public void setShooterSpeed(double setpoint) {
    m_shooterPID.setSetpoint(setpoint, ControlType.kVelocity);
  }
  public void setAgitatorSpeed(double speed) {
    m_AgitatorMotor.set(speed);
  }

  public void setLeftRollerSpeed(double speed) {
    m_leftRollerMotor.set(speed);
  }

  // public Command runShooter() {
  //   return this.startEnd(
  //     () -> this.setShooterSpeed(ShooterConstants.kShooterLeftSpeed),
  //     m_shooterLeftMotor::stopMotor);
  // }
  // public Command runAgitator() {
  //   return this.startEnd(
  //     () -> this.setAgitatorSpeed(ShooterConstants.kAgitatorSpeed),
  //     m_topAgitatorMotor::stopMotor);
  // }

  public Command agitate() {
    return this.run(() -> { this.setLeftRollerSpeed(ShooterConstants.kLeftRollerSpeed); });
  }

  public Command stopAgitator() {
    return this.runOnce(m_leftRollerMotor::stopMotor);
  }

  public Command runShooterAndAgitate() {
    return Commands.sequence(
      this.run(() -> {this.setShooterSpeed(ShooterConstants.kShooterLeftSpeed);}).withTimeout(0.5),
      this.run(() -> {
        this.setShooterSpeed(ShooterConstants.kShooterLeftSpeed);
        this.setAgitatorSpeed(ShooterConstants.kAgitatorSpeed);
        this.setLeftRollerSpeed(ShooterConstants.kLeftRollerSpeed);
      })
    );
  }

  public Command stopShooterAndAgitator() {
    return this.runOnce(() -> {
      this.m_shooterLeftMotor.stopMotor();
      this.m_AgitatorMotor.stopMotor();
      this.m_leftRollerMotor.stopMotor();});
  }


  public Command reverseShooterAndAgitate() {
    return this.startEnd(
      () -> {
        this.setShooterSpeed(ShooterConstants.kReverseShooterSpeed);
        this.setAgitatorSpeed(ShooterConstants.kReverseAgitatorSpeed);
        this.setLeftRollerSpeed(ShooterConstants.kReverseLeftRollerSpeed);
      },
      () -> {
        this.m_shooterLeftMotor.stopMotor();
        this.m_AgitatorMotor.stopMotor();
        this.m_leftRollerMotor.stopMotor();
      }
    );
  }

  public Command reverseShooterAndAgitateInAuto() {
    return this.startEnd(
      () -> {
        this.setShooterSpeed(ShooterConstants.kReverseShooterSpeed);
        this.setAgitatorSpeed(ShooterConstants.kReverseAgitatorSpeed);
      },
      () -> {
        this.m_shooterLeftMotor.stopMotor();
        this.m_AgitatorMotor.stopMotor();
      }
    );
  }

  public Command rangedShooting(DriveSubsystem drive) {
    return Commands.sequence(
      this.run(() -> {
        double distanceToTarget = drive.distanceToTarget(Constants.isBlueAlliance.get() ? Constants.FieldConstants.kBlueHubLocation : Constants.FieldConstants.kRedHubLocation);
        SmartDashboard.putNumber("Distance to Hub", distanceToTarget);
        double desiredShooterSpeed = FlywheelLookup.getRpmForDistance(distanceToTarget);
        this.setShooterSpeed(desiredShooterSpeed);}).withTimeout(0.5),
      this.run(() -> {
        double distanceToTarget = drive.distanceToTarget(Constants.isBlueAlliance.get() ? Constants.FieldConstants.kBlueHubLocation : Constants.FieldConstants.kRedHubLocation);
        if  (distanceToTarget >= 1.967) {
          SmartDashboard.putNumber("Distance to Hub", distanceToTarget);
          double desiredShooterSpeed = FlywheelLookup.getRpmForDistance(distanceToTarget);
          this.setShooterSpeed(desiredShooterSpeed);
          this.setAgitatorSpeed(ShooterConstants.kAgitatorSpeed);
          this.setLeftRollerSpeed(ShooterConstants.kLeftRollerSpeed);
        }
      })
    );
  }

  // public Command reverseShooter() {
  //   return this.startEnd(() -> this.setShooterSpeed(ShooterConstants.kReverseShooterSpeed),
  //   () -> this.setShooterSpeed(0));
  // }
  // public Command reverseAgitator() {
  //   return this.startEnd(() -> this.setAgitatorSpeed(ShooterConstants.kReverseAgitatorSpeed),
  //   () -> this.setAgitatorSpeed(0));
  // }


    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooterSpeed", m_shooterLeftMotor.getEncoder().getVelocity());

    // if (m_bottomAgitatorMotor.getOutputCurrent()>= ShooterConstants.kBottomAgitatorSmartCurrentLimit){
    //   m_bottomAgitatorMotor.set(kReverseAgitatorSpeed / 2);
    // }
    // else if (m_bottomAgitatorMotor.getOutputCurrent() < ShooterConstants.kBottomAgitatorSmartCurrentLimit){
    //   m_bottomAgitatorMotor.set(kTopAgitatorSpeed);
    // }
  }

}


