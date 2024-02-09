// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExtendAmpAssistCommand;
import frc.robot.commands.NoteIntakeFromSourceCommand;
import frc.robot.commands.RetractAmpAssistCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.ToggleWallSpacerCommand;
import frc.robot.subsystems.AmpAssistSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WallSpacerSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterLocation;

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
  // private final WallSpacerSubsystem m_wallSpacerSubsystem = new WallSpacerSubsystem();
  // private final AmpAssistSubsystem m_AmpAssistSubsystem = new AmpAssistSubsystem();
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
  JoystickButton m_reverseIntakeButton = new JoystickButton(m_rightJoystick, RobotMap.kReverseIntake);

  // co-pilot box buttons
  JoystickButton m_wallSpacerButton = new JoystickButton(m_buttonBox, RobotMap.kToggleWallSpacerButton);
  JoystickButton m_extendAmpAssistButton =new JoystickButton(m_buttonBox, RobotMap.kExtendAmpAssistPleaseButton);
  JoystickButton m_retractAmpAssistButton =new JoystickButton(m_buttonBox, RobotMap.kRetractAmpAssistPleaseButton);
  JoystickButton m_ampShootButton =new JoystickButton(m_buttonBox, RobotMap.kAmpShooterVelocityControl);
  JoystickButton m_trapShootButton =new JoystickButton(m_buttonBox, RobotMap.kTrapShooterVelocityControl);
  JoystickButton m_centerSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kCenterSpeakerShooterVelocityControl);
  JoystickButton m_rightSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kRightSpeakerShooterVelocityControl);
  JoystickButton m_leftSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kLeftSpeakerShooterVelocityControl);
  JoystickButton m_sourceReceiveButton =new JoystickButton(m_buttonBox, RobotMap.kSourceShooterVelocityControl);
  JoystickButton m_sourceLoadButton = new JoystickButton(m_buttonBox, RobotMap.kLoadFromSourceButton);

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

    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
          () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
          () -> m_robot.isTeleopEnabled()));

    m_intakeButton.onTrue(new InstantCommand(() -> m_intakeSubsystem.toggleIntakeSubsystem()));
    m_reverseIntakeButton.whileTrue(new ReverseIntakeCommand(m_intakeSubsystem));

    // m_extendAmpAssistButton.onTrue(new ExtendAmpAssistCommand(m_AmpAssistSubsystem));
    // m_retractAmpAssistButton.onTrue(new RetractAmpAssistCommand(m_AmpAssistSubsystem));
    // m_wallSpacerButton.onTrue(new ToggleWallSpacerCommand(m_wallSpacerSubsystem));

    m_fireButtonRT.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Generic));
    m_centerSpeakerShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.CenterSpeaker));
    m_leftSpeakerShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.LeftSpeaker));
    m_rightSpeakerShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.RightSpeaker));
    m_ampShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Amp));
    m_trapShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Trap));
    m_sourceReceiveButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.SourceLoad));
    // m_sourceLoadButton.onTrue(new NoteIntakeFromSourceCommand(m_shooterSubsystem, m_intakeSubsystem));

    new Trigger(m_intakeSubsystem::isNotePresent)
      // .and(m_intakeSubsystem::isPickingUpNote)
      .onTrue(new InstantCommand(() -> m_intakeSubsystem.stopMotor()));
  }

  public void smartDashboardInit() {
      SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
      // m_driveSubsystem.smartDashboardInit();
      m_shooterSubsystem.smartDashboardInit();
      m_intakeSubsystem.smartDashboardInit();
      // m_AmpAssistSubsystem.smartDashboardInit();
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
          // m_AmpAssistSubsystem.smartDashboardUpdate();
          // m_climbSubsystem.smartDashboardUpdate();
      }, 1, 0.303);
      // m_robot.addPeriodic(() -> {
      //     m_driveSubsystem.checkRelativeEncoderToAbsoluteEncoder();
      // }, 1, 0.107);
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
