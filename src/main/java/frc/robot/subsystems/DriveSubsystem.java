// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.Navx;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;


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
  // private final Pigeon2 m_gyro = new Pigeon2(30);



  private final Navx m_gyro = new Navx(0);

  private final Field2d field2d = new Field2d();



  private final SwerveDrivePoseEstimator m_poseEstimator;
    private final PIDController m_limeLightAimPidController = new PIDController(0, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);


m_gyro.enableOptionalMessages(true, true, false, false, false, true, false, false, false, false);
//error here, added arguments to fix, change later?


zeroHeading();
m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getAngle(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    SmartDashboard.putNumber("Aiming KP", 0.02);
    SmartDashboard.putNumber("Aiming KI", 0);
    SmartDashboard.putNumber("Aiming KD", 0.0005);
  SmartDashboard.putNumber("AimingTolerance", 0.5);

  
  // Load the RobotConfig from the GUI settings. You should probably
  // store this in your Constants file
  RobotConfig config;
  try{
    config = RobotConfig.fromGUISettings();
  } catch (Exception e) {
    // Handle exception as needed
    e.printStackTrace();
    config = null;
  }

  // Configure AutoBuilder last
  AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                  new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements 
  );
}

  @Override
  public void periodic() {
        Rotation2d targetAngle = headingToTarget(getPose(), FieldConstants.kHubLocation);

    SmartDashboard.putNumber("targetAngle", targetAngle.getDegrees());
    SmartDashboard.putNumber("gyroAngle", getHeading());
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
      getAngle(),       
      new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        
        boolean useMegaTag2 = true;
        boolean doRejectUpdate = false;
        if(!useMegaTag2) {
          LimelightHelpers.PoseEstimate megaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
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
          LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
          //This thing below gave some issues. Changed to getRotation2d().getDegrees() instead of getAnglularVel() which gave some errors, build successful
          if (Math.abs(getHeading()) > 720) { 
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

        //coordinate system

  public Rotation2d headingToTarget(Pose2d robotPose, Pose2d targetPose){
    Translation2d translation = targetPose.getTranslation().minus(robotPose.getTranslation());
    return translation.getAngle();
  }
  public Rotation2d headingErrorToTarget(Pose2d robotPose, Pose2d targetPose){
      Rotation2d targetAngle = headingToTarget(getPose(), FieldConstants.kHubLocation);
      return targetAngle.minus(robotPose.getRotation());
}
  
//4.625594m for x distance to hub
//4.034536 for y distance to hub

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
      getAngle(), 
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

  public Command targetTrack(DoubleSupplier xSpeed, DoubleSupplier ySpeed){
    return this.run(() -> {
      Rotation2d targetAngleError = headingErrorToTarget(getPose(), FieldConstants.kHubLocation);
      double rotSpeed = m_limeLightAimPidController.calculate(targetAngleError.getDegrees(), 0);
      if (m_limeLightAimPidController.atSetpoint()){
        //coordinate system
        drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), 0, true);
      } else {
        drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), -rotSpeed, false);
      }
    });


    
  }
  public Command limeLightAim(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed) {
    return this.run(() -> {
      LimelightHelpers.PoseEstimate megaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
      
      if(megaTag1 != null && megaTag1.tagCount > 0){
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
    m_gyro.resetYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return MathUtil.inputModulus(m_gyro.getRotation2d().getDegrees(), -180.0, 180.0);
  }
  public Rotation2d getAngle() {
    return m_gyro.getRotation2d();

  }

private ChassisSpeeds getRobotRelativeSpeeds() {
  SwerveModuleState[] measuredStates = new SwerveModuleState[] {
    m_frontLeft.getState(),
    m_frontRight.getState(),
    m_rearLeft.getState(),
    m_rearRight.getState()
  };
  return DriveConstants.kDriveKinematics.toChassisSpeeds(measuredStates);
}
private void driveRobotRelative(ChassisSpeeds speeds) {
  var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
}
