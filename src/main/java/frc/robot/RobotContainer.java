// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.ToggleWallSpacerCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WallSpacerSubsystem;
import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static double kDriveYExponent = 2;
  private static double kDriveXExponent = 2;
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final WallSpacerSubsystem m_wallSpacerSubsystem = new WallSpacerSubsystem();
  // private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  Robot m_robot;

  Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
  Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
  Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);

  Conditioning m_driveXConditioning = new Conditioning();
  Conditioning m_driveYConditioning = new Conditioning();
  Conditioning m_turnConditioning = new Conditioning();
  double m_speedMultiplier = 0.70;
  
  // Driver Buttons
  JoystickButton m_intakeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightToggleIntakeButton);
  JoystickButton m_fireButtonRT = new JoystickButton(m_rightJoystick, RobotMap.kRightTriggerFire);

  // co-pilot box buttons
  JoystickButton m_wallSpacerButton = new JoystickButton(m_buttonBox, RobotMap.kToggleWallSpacerButton);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    m_driveXConditioning.setDeadband(0.15);
    m_driveXConditioning.setExponent(kDriveXExponent);
    m_driveYConditioning.setDeadband(0.15);
    m_driveYConditioning.setExponent(kDriveYExponent);
    m_turnConditioning.setDeadband(0.2);
    m_turnConditioning.setExponent(1.4);

    // Configure the trigger bindings
    configureBindings();

    smartDashboardInit();
    registerSmartDashboardCalls();
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
 
    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
          () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
          () -> m_robot.isTeleopEnabled()));

    // m_intakeButton.onTrue(new ToggleIntakeCommand(m_intakeSubsystem));
    m_intakeButton.onTrue(new InstantCommand(() -> m_intakeSubsystem.toggleIntakeSubsystem()));
    m_wallSpacerButton.onTrue(new ToggleWallSpacerCommand(m_wallSpacerSubsystem));
    m_fireButtonRT.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem));
  }

  public void smartDashboardInit() {
      SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
      m_driveSubsystem.smartDashboardInit();
      m_shooterSubsystem.smartDashboardInit();
      m_intakeSubsystem.smartDashboardInit();
      // m_climbSubsystem.smartDashboardInit();
  }

  public void registerSmartDashboardCalls() {
      m_robot.addPeriodic(() -> {
          // m_driveSubsystem.smartDashboardUpdate();
          smartDashboardUpdate();
      }, 2, 0.502);
      m_robot.addPeriodic(() -> {
          m_shooterSubsystem.smartDashboardUpdate();
          m_intakeSubsystem.smartDashboardUpdate();
          // m_climbSubsystem.smartDashboardUpdate();
      }, 1, 0.303);
  }

  public void smartDashboardUpdate() {
    m_speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", m_speedMultiplier);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public double getDriveXInput()
  {
    // We getY() here because of the FRC coordinate system being turned 90 degrees
    return m_driveXConditioning.condition(-m_leftJoystick.getY())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getDriveYInput()
  {
    // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
    return m_driveYConditioning.condition(-m_leftJoystick.getX())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getTurnInput()
  {
    return m_turnConditioning.condition(-m_rightJoystick.getX())
            * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond
            * m_speedMultiplier;
  }
}
