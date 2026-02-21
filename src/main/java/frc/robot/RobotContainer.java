// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.IntakeSetpoints;

import com.pathplanner.lib.auto.AutoBuilder;


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
  
  
  
  
      // Drive suppliers
    DoubleSupplier driverX = () -> -leftController.getRawAxis(1); // Y-axis joystick
    DoubleSupplier driverY = () -> -leftController.getRawAxis(0); // X-axis joystick
    DoubleSupplier angleX = () -> rightController.getRawAxis(0); // X-axis joystick
    DoubleSupplier angleY = () -> -rightController.getRawAxis(1); // Y-axis joystick
  
    private final SendableChooser<Command> m_autoChooser;
    

  
    public RobotContainer() {
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
        .whileTrue(      
          new ParallelCommandGroup(
            m_shooter.runShooter(),
            m_shooter.runShooter2(),
            m_shooter.runAgitator()
        )
        );

      new JoystickButton(leftController, 1)
        .whileTrue(  
          new ParallelCommandGroup(
            m_intake.runIntake(),
            m_intake.feedFlipper()
            ));


      new JoystickButton(rightController, 2)
        .whileTrue(m_robotDrive.targetTrack(
          () -> { return -MathUtil.applyDeadband(leftController.getY(), OIConstants.kDriveDeadband); },
          () -> { return -MathUtil.applyDeadband(leftController.getX(), OIConstants.kDriveDeadband); }
        ));


      new JoystickButton(leftController, 4)
        .whileTrue(m_intake.stowFlipper());



  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    
  }
}
