// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.Navx;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private final Navx m_gyro = new Navx(0);

  private final Field2d field2d = new Field2d();

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final PIDController m_limeLightAimPidController = new PIDController(0, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    m_gyro.enableOptionalMessages(true, true, false, false, false, false, false, false, false, false);

    LimelightHelpers.SetIMUAssistAlpha("limelight-shooter", 0.001);
    LimelightHelpers.SetIMUMode("limelight-shooter", 1);

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
    // if (!RobotState.isEnabled()) {
    //   if(Constants.isBlueAlliance.get()){
    //     resetPose(new Pose2d(3, 4.1, Rotation2d.kZero));
    //   } else {
    //     resetPose(new Pose2d(14,4.8, Rotation2d.k180deg));
    //   }
    // }
    SmartDashboard.putNumber("gyroAngle", getAngle().getDegrees());
    double aimingKP = SmartDashboard.getNumber("Aiming KP", 0);
    double aimingKI = SmartDashboard.getNumber("Aiming KI", 0);
    double aimingKD = SmartDashboard.getNumber("Aiming KD", 0);
    double aimingTolerance = SmartDashboard.getNumber("AimingTolerance", 0);
    m_limeLightAimPidController.setP(aimingKP);
    m_limeLightAimPidController.setI(aimingKI);
    m_limeLightAimPidController.setD(aimingKD);
    m_limeLightAimPidController.setTolerance(aimingTolerance);

    LimelightHelpers.SetIMUMode("limelight-shooter", 4);

    // Update the odometry in the periodic block
    m_poseEstimator.update(
      getAngle(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      });

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight-shooter", getAngle().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
    if (Math.abs(m_gyro.getAngularVel()[2].in(DegreesPerSecond)) > 720) {
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

  public Rotation2d headingToTarget(Pose2d robotPose, Pose2d targetPose){
    Translation2d translation = targetPose.getTranslation().minus(robotPose.getTranslation());
    return translation.getAngle();
  }

  public Rotation2d headingErrorToTarget(Pose2d robotPose, Pose2d targetPose){
      Rotation2d targetAngle = headingToTarget(getPose(), targetPose);
      return targetAngle.minus(robotPose.getRotation());
  }

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

    ChassisSpeeds speeds = new ChassisSpeeds(
      xSpeedDelivered, ySpeedDelivered, rotDelivered);
    boolean isFlipped = !Constants.isBlueAlliance.get();
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          isFlipped
            ? m_poseEstimator.getEstimatedPosition().getRotation().plus(Rotation2d.k180deg)
            : m_poseEstimator.getEstimatedPosition().getRotation())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command targetTrack(DoubleSupplier xSpeed, DoubleSupplier ySpeed){
    return this.run(() -> {
      Pose2d hubLocation = Constants.isBlueAlliance.get() ? FieldConstants.kBlueHubLocation : FieldConstants.kRedHubLocation;
      Rotation2d targetAngleError = headingErrorToTarget(getPose(), hubLocation);
      double rotSpeed = m_limeLightAimPidController.calculate(targetAngleError.getDegrees(), 0);
      if (m_limeLightAimPidController.atSetpoint()){
        //coordinate system
        drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), 0, true);
      } else {
        drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), -rotSpeed, true);
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

  public Rotation2d getAngle() {
    Rotation2d angle = m_gyro.getRotation2d();
    return angle;
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
