// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.DoubleSupplier;
//import frc.robot.Constants.IntakeSetpoints;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
// drive subsytem was private before
  // The driver's controller



  // Controller
  public static final Joystick leftController = new Joystick(1);
  public static final Joystick rightController = new Joystick(0);

  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final ClimberSubsystem m_climber = new ClimberSubsystem();

    // Drive suppliers
    DoubleSupplier driverX = () -> -leftController.getRawAxis(1); // Y-axis joystick
    DoubleSupplier driverY = () -> -leftController.getRawAxis(0); // X-axis joystick
    DoubleSupplier angleX = () -> rightController.getRawAxis(0); // X-axis joystick
    DoubleSupplier angleY = () -> -rightController.getRawAxis(1); // Y-axis joystick

    private final SendableChooser<Command> m_autoChooser;
    private boolean wasAutonomousEnabled = false;
    private boolean hasDashboardSwitched = false;

    public RobotContainer() {
     // Register Named Commands
    // NamedCommands.registerCommand("autoBalance", m_robotDrive.run(() -> m_robotDrive.setX()));
      NamedCommands.registerCommand("runShooter", m_shooter.rangedShooting(m_robotDrive));
      NamedCommands.registerCommand("reverseShooter", m_shooter.reverseShooterAndAgitateInAuto());
      NamedCommands.registerCommand("dropFlipper", m_intake.dropFlipper().withTimeout(3.0));
      NamedCommands.registerCommand("runIntake", m_intake.runIntake());
      NamedCommands.registerCommand("stowFlipper", m_intake.stowFlipper());
      NamedCommands.registerCommand("targetTrack", m_robotDrive.targetTrack(() -> { return 0; }, () -> { return 0; }).withTimeout(2.0));
      NamedCommands.registerCommand("stopDrive", new RunCommand(() -> { m_robotDrive.drive(0, 0, 0, false); }, m_robotDrive));
      NamedCommands.registerCommand("feed", m_intake.runIntakeAndDropFlipper());

      // NamedCommands.registerCommand("climberRaise", m_climber.climberRaise());
      NamedCommands.registerCommand("climberPull", m_climber.climberLockout());
      NamedCommands.registerCommand("climberStow", m_climber.climberStow());
      //have to be added to pathplanner still

      // Do all other initialization
      // configureButtonBindings();

      LimelightHelpers.setRewindEnabled("limelight-shooter", true);

      m_autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
      // Configure the button bindings
      configureButtonBindings();

      // Configure default commands
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(leftController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(leftController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rightController.getX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

      m_intake.setDefaultCommand(m_intake.dropFlipper());
      m_shooter.setDefaultCommand(m_shooter.stopShooterAndAgitator());
      m_climber.setDefaultCommand(m_climber.climberStop());

    }

    public void teleopInit(){
      HubShiftUtil.initialize();
      LimelightHelpers.triggerRewindCapture("limelight-shooter", 40);
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
      // Default command, normal field-relative drive
      // Joystick drive command (driver and operator)

      //Shooter Buttons and Intake Buttons
      new JoystickButton(rightController, 1)
        .whileTrue(m_shooter.rangedShooting(m_robotDrive));

      new JoystickButton(leftController, 1)
        .whileTrue(m_intake.runIntakeAndDropFlipper());

      new JoystickButton(rightController, 3)
        .whileTrue(m_shooter.reverseShooterAndAgitate());

      new JoystickButton(rightController, 4)
        .whileTrue( m_robotDrive.targetTrack(
            () -> { return -MathUtil.applyDeadband(leftController.getY(), OIConstants.kDriveDeadband); },
            () -> { return -MathUtil.applyDeadband(leftController.getX(), OIConstants.kDriveDeadband);}));

      new JoystickButton(leftController, 4)
        .toggleOnTrue(m_intake.stowFlipper());

      new JoystickButton(leftController, 2)
        .whileTrue(m_intake.reverseIntake());

      new JoystickButton(leftController, 3)
        .whileTrue(m_robotDrive.setXCommand());

      new JoystickButton(leftController, 6)
        .onTrue(Commands.runOnce(m_robotDrive::zeroHeading).ignoringDisable(true).withName("zeroGyro"));

      new JoystickButton(rightController,6)
        .onTrue(Commands.parallel(
          m_intake.stowFlipper(),
          Commands.sequence(Commands.waitSeconds(1.5),
                            m_climber.climberLockout())));

      new JoystickButton(rightController,5)
        .onTrue(m_climber.climberStow());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    wasAutonomousEnabled = true;
    return m_autoChooser.getSelected();
  }

  public void robotPeriodic() {
    if (!DriverStation.isAutonomousEnabled() && wasAutonomousEnabled && !hasDashboardSwitched){
      Elastic.selectTab("Teleoperated");
      hasDashboardSwitched = true;
    }
    if (DriverStation.isAutonomousEnabled()) {
      SmartDashboard.putNumber(
        "Remaining Time In Current Shift",
        DriverStation.getMatchTime());
    } else {
      SmartDashboard.putNumber(
        "Remaining Time In Current Shift",
        HubShiftUtil.getOfficialShiftInfo().remainingTime());
    }
    SmartDashboard.putBoolean(
        "Our Hub is Active?", HubShiftUtil.getOfficialShiftInfo().active());
    SmartDashboard.putString(
        "Current Shift", HubShiftUtil.getOfficialShiftInfo().currentShift().name());
    SmartDashboard.putString(
        "Auto Winner",HubShiftUtil.getFirstActiveAlliance() == Alliance.Blue ? "Red" : "Blue");
  }
}
