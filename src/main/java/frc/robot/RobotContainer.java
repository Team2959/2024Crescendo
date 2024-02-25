// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.DetectNoteIntakeFromFloorCommand;
import frc.robot.commands.ExtendAmpAssistCommand;
import frc.robot.commands.ExtendWallSpacerCommand;
import frc.robot.commands.FeedNoteIntoShooterCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.commands.NoteIntakeFromFloorCommand;
import frc.robot.commands.NoteIntakeFromSourceCommand;
import frc.robot.commands.RetractAmpAssistCommand;
import frc.robot.commands.RetractWallSpacerCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.subsystems.AmpAssistSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterLocation;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WallSpacerSubsystem;

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
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final WallSpacerSubsystem m_wallSpacerSubsystem = new WallSpacerSubsystem();
  private final AmpAssistSubsystem m_AmpAssistSubsystem = new AmpAssistSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  // public final Vision m_vision = new Vision();

  // private SendableChooser<Command> m_autoChooser = Autos.sendableChooser(this);
  // Path Planner
  private SendableChooser<Command> m_autoChooser;

  Robot m_robot;

  Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
  Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
  Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);

  Conditioning m_driveXConditioning = new Conditioning();
  Conditioning m_driveYConditioning = new Conditioning();
  Conditioning m_turnConditioning = new Conditioning();
  double m_speedMultiplier = 0.80;
  public double m_delayTimeForShooter = 0.5;

  // Driver Buttons
  JoystickButton m_intakeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightToggleIntakeButton);
  JoystickButton m_fireButtonRT = new JoystickButton(m_rightJoystick, RobotMap.kRightTriggerFire);
  JoystickButton m_lockWheeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightLockWheels);
  JoystickButton m_resetNavX = new JoystickButton(m_rightJoystick, RobotMap.kRightResetNavXButton);
  JoystickButton m_extendClimbButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftExtendClimbButton);
  JoystickButton m_retractClimbButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftRetractClimbButton);

  // co-pilot box buttons
  JoystickButton m_extendAmpAssistButton =new JoystickButton(m_buttonBox, RobotMap.kExtendAmpAssistPleaseButton);
  JoystickButton m_retractAmpAssistButton =new JoystickButton(m_buttonBox, RobotMap.kRetractAmpAssistPleaseButton);
  JoystickButton m_ampShootButton =new JoystickButton(m_buttonBox, RobotMap.kAmpShooterVelocityControl);
  JoystickButton m_trapShootButton =new JoystickButton(m_buttonBox, RobotMap.kTrapShooterVelocityControl);
  JoystickButton m_centerSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kCenterSpeakerShooterVelocityControl);
  JoystickButton m_rightSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kRightSpeakerShooterVelocityControl);
  JoystickButton m_leftSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kLeftSpeakerShooterVelocityControl);
  JoystickButton m_sourceLoadButton = new JoystickButton(m_buttonBox, RobotMap.kLoadFromSourceButton);
  JoystickButton m_reverseIntakeButton = new JoystickButton(m_buttonBox, RobotMap.kReverseIntake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    m_driveXConditioning.setDeadband(0.15);
    m_driveXConditioning.setExponent(kDriveXExponent);
    m_driveYConditioning.setDeadband(0.15);
    m_driveYConditioning.setExponent(kDriveYExponent);
    m_turnConditioning.setDeadband(0.2);
    m_turnConditioning.setExponent(1.4);

    registerPathPlannerNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    // m_autoChooser = AutoBuilder.buildAutoChooser("Center two note");
    SmartDashboard.putData("Auto/Routine", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();

    smartDashboardInit();
    registerSmartDashboardCalls();
  }

  private void registerPathPlannerNamedCommands()
  {
    // Register Named Commands - things to do at marked points, e.g. start intake, shoot, etc
    // Must match names in PathPlanner!
    NamedCommands.registerCommand("grabNoteFromFloor", new NoteIntakeFromFloorCommand(m_intakeSubsystem));

    NamedCommands.registerCommand("startShooterCenter", new InstantCommand(() ->
        m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.CenterSpeaker), m_shooterSubsystem));
    NamedCommands.registerCommand("startShooterLeft", new InstantCommand(() ->
        m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.LeftSpeaker), m_shooterSubsystem));
    NamedCommands.registerCommand("startShooterRight", new InstantCommand(() ->
        m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.RightSpeaker), m_shooterSubsystem));
    NamedCommands.registerCommand("startShooterTrap", new InstantCommand(() ->
        m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.Trap), m_shooterSubsystem));

    NamedCommands.registerCommand("waitAndFeedNoteIntoShooter",
        new WaitCommand(m_delayTimeForShooter)
        .andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem)));
    NamedCommands.registerCommand("feedNoteIntoShooter", new FeedNoteIntoShooterCommand(m_intakeSubsystem));

    NamedCommands.registerCommand("stopShooter", new InstantCommand(() ->
        m_shooterSubsystem.stopShooterMotor(), m_shooterSubsystem));
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

    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
          () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
          () -> m_robot.isTeleopEnabled()));
    m_lockWheeButton.whileTrue(new LockWheelsCommand(m_driveSubsystem));
    m_resetNavX.onTrue(new InstantCommand(() -> {m_driveSubsystem.resetNavX();}));

    // All the Note delivery commands
    m_fireButtonRT.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Generic)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));

    m_centerSpeakerShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.CenterSpeaker)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));
    m_leftSpeakerShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.LeftSpeaker)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));
    m_rightSpeakerShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.RightSpeaker)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));
    m_ampShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Amp)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));
    m_trapShootButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Trap)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));

    // Pick up Note from Floor
    m_intakeButton.onTrue(new NoteIntakeFromFloorCommand(m_intakeSubsystem)
        .andThen(new DetectNoteIntakeFromFloorCommand(m_intakeSubsystem)));
    m_reverseIntakeButton.whileTrue(new ReverseIntakeCommand(m_intakeSubsystem));

    // Load Note From Source
    m_sourceLoadButton.onTrue(
        new NoteIntakeFromSourceCommand(m_shooterSubsystem, m_intakeSubsystem)
        .alongWith(new ExtendWallSpacerCommand(m_wallSpacerSubsystem)));

    // Amp Assist
    m_extendAmpAssistButton.onTrue(new ExtendAmpAssistCommand(m_AmpAssistSubsystem));
    m_retractAmpAssistButton.onTrue(
      new RetractAmpAssistCommand(m_AmpAssistSubsystem)
      .alongWith(new RetractWallSpacerCommand(m_wallSpacerSubsystem)));

    // Climb
    m_extendClimbButton.onTrue(new ClimbExtendCommand(m_climbSubsystem));
    m_retractClimbButton.onTrue(new ClimbRetractCommand(m_climbSubsystem));

    // Autonomous Trigger to stop finish floor intake
    new Trigger(m_intakeSubsystem::isNotePresent)
      .and(m_robot::isAutonomousEnabled)
      .onTrue(new DetectNoteIntakeFromFloorCommand(m_intakeSubsystem));
  }

  public void smartDashboardInit() {
      SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
      m_driveSubsystem.smartDashboardInit();
      m_shooterSubsystem.smartDashboardInit();
      m_intakeSubsystem.smartDashboardInit();
      m_wallSpacerSubsystem.smartDashboardInit();
      m_AmpAssistSubsystem.smartDashboardInit();
      m_climbSubsystem.smartDashboardInit();
  }

  public void registerSmartDashboardCalls() {
      m_robot.addPeriodic(() -> {
          m_driveSubsystem.smartDashboardUpdate();
          smartDashboardUpdate();
      }, 2, 0.502);
      m_robot.addPeriodic(() -> {
          m_shooterSubsystem.smartDashboardUpdate();
          // m_intakeSubsystem.smartDashboardUpdate();
          m_wallSpacerSubsystem.smartDashboardUpdate();
          m_AmpAssistSubsystem.smartDashboardUpdate();
          m_climbSubsystem.smartDashboardUpdate();
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
    return m_autoChooser.getSelected();
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
// example decorator pattern for commands
    // m_fireButtonRT.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Generic)
    //   .alongWith(
    //     new WaitCommand(0.25)
    //     .andThen(new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem))
    //     .until(m_intakeSubsystem::isNotePresent)
    //     .finallyDo(() -> m_intakeSubsystem.stopMotor())));

    // new Trigger(m_intakeSubsystem::isNotePresent)
    //   .and(m_intakeSubsystem::isPickingUpNote)
    //   .onTrue(new InstantCommand(() -> m_intakeSubsystem.stopMotor()));
