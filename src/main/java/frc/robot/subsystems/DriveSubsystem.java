// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(30);

  private final Field2d field2d = new Field2d();

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    private final PIDController m_limeLightAimPidController = new PIDController(0, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putNumber("Aiming KP", 0.02);
    SmartDashboard.putNumber("Aiming KI", 0);
    SmartDashboard.putNumber("Aiming KD", 0.0005);
    SmartDashboard.putNumber("AimingTolerance", 0.5);
  }

  @Override
  public void periodic() {
    double aimingKP = SmartDashboard.getNumber("Aiming KP", 0);
    double aimingKI = SmartDashboard.getNumber("Aiming KI", 0);
    double aimingKD = SmartDashboard.getNumber("Aiming KD", 0);
    double aimingTolerance = SmartDashboard.getNumber("AimingTolerance", 0);
    m_limeLightAimPidController.setP(aimingKP);
    m_limeLightAimPidController.setI(aimingKI);
    m_limeLightAimPidController.setD(aimingKD);
    m_limeLightAimPidController.setTolerance(aimingTolerance);


    // Update the odometry in the periodic block
    m_poseEstimator.update(
      m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        
        boolean useMegaTag2 = true;
        boolean doRejectUpdate = false;
        if(!useMegaTag2) {
          LimelightHelpers.PoseEstimate megaTag1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-shooter");
          if (megaTag1 == null) {
            doRejectUpdate = true;
          } else if (megaTag1.tagCount == 1 && megaTag1.rawFiducials.length == 1) {
            doRejectUpdate = megaTag1.rawFiducials[0].ambiguity > 0.7 || megaTag1.rawFiducials[0].distToCamera > 3.0;
            
          } else if(megaTag1.tagCount == 0 ){
            doRejectUpdate = true;

          }
          if(!doRejectUpdate ){
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,9999999));
            m_poseEstimator.addVisionMeasurement(megaTag1.pose, 
            megaTag1.timestampSeconds);
          }
        } else {
          LimelightHelpers.SetRobotOrientation("limelight-shooter", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
          LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-shooter");
          if(Math.abs(m_gyro.getAngularVelocityZWorld().refresh().getValueAsDouble()) > 720) {
            doRejectUpdate = true;
          }
          if (megaTag2 == null) {
            doRejectUpdate = true;
          } else if(megaTag2.tagCount == 0 ){
            doRejectUpdate = true;
          }
          if(!doRejectUpdate ){
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
            m_poseEstimator.addVisionMeasurement(megaTag2.pose, 
            megaTag2.timestampSeconds);
          }
        }

        
      // adding a field map to the smart dashboard
      field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
      SmartDashboard.putData("Field2d", field2d);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
         m_poseEstimator.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  public Command limeLightAim(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed) {
    return this.run(() -> {
      LimelightHelpers.PoseEstimate megaTag1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-shooter");
      
      if(megaTag1 != null && megaTag1.tagCount > 0){
        Pose2d targetPose = megaTag1.pose;
        Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
        // double xError = targetPose.getX() - currentPose.getX();
        // double yError = targetPose.getY() - currentPose.getY();
        // double rotError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
        double rotError = LimelightHelpers.getTX("limelight-shooter");
        double rotSpeed = m_limeLightAimPidController.calculate(rotError, 0);
    

        // Simple proportional control for demonstration purposes
        if (m_limeLightAimPidController.atSetpoint()){
          drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zSpeed.getAsDouble(), true); 
        } else { 
          drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed, false);
        }
      } else {
        drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zSpeed.getAsDouble(), true); // Stop if no target is found
      }
    });
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }


  
}
