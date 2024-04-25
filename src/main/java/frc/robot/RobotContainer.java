// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cwtech.util.AprilTagPID;
import cwtech.util.Bling;
import cwtech.util.Conditioning;
import cwtech.util.Bling.BlingMessage;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignWithAmpCommand;
import frc.robot.commands.AlignWithTrapCommand;
import frc.robot.commands.AmpDirectDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbLatchCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.DetectNoteIntakeFromFloorCommandForAutonomous;
import frc.robot.commands.ExtendAmpAssistCommand;
import frc.robot.commands.FeedNoteIntoShooterCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.commands.NoteIntakeFromFloorCommand;
import frc.robot.commands.RetractAmpAssistCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.ShooterVelocityCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.subsystems.AmpAssistSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final AmpAssistSubsystem m_AmpAssistSubsystem = new AmpAssistSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final Bling m_bling = new Bling();
  private final AprilTagPID m_AprilTagPID = new AprilTagPID(m_driveSubsystem);

  private SendableChooser<Command> m_autoChooser;
  // private SendableChooser<autoStartPosition> m_angleStartChooser;

  Robot m_robot;

  Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
  Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
  Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);

  Conditioning m_driveXConditioning = new Conditioning();
  Conditioning m_driveYConditioning = new Conditioning();
  Conditioning m_turnConditioning = new Conditioning();
  double m_speedMultiplier = 0.3;
  public double m_delayTimeForShooter = 0.75;
  // Driver Buttons
  JoystickButton m_intakeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightToggleIntakeButton);
  JoystickButton m_fireButtonRT = new JoystickButton(m_rightJoystick, RobotMap.kRightTriggerFire);
  JoystickButton m_lockWheeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightLockWheels);
  JoystickButton m_resetNavX = new JoystickButton(m_rightJoystick, RobotMap.kRightResetNavXButton);
  JoystickButton m_extendClimbButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftExtendClimbButton);
  JoystickButton m_retractClimbButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftRetractClimbButton);
  JoystickButton m_latchClimbButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftLatchClimbButton);
  JoystickButton m_trapAlignButton = new JoystickButton(m_rightJoystick, RobotMap.kTrapAlignButton);
  JoystickButton m_ampAlignButton = new JoystickButton(m_rightJoystick, RobotMap.kRightAmpAlign);

  // co-pilot box buttons
  JoystickButton m_extendAmpAssistButton =new JoystickButton(m_buttonBox, RobotMap.kExtendAmpAssistPleaseButton);
  JoystickButton m_retractAmpAssistButton =new JoystickButton(m_buttonBox, RobotMap.kRetractAmpAssistPleaseButton);
  JoystickButton m_trapShootButton =new JoystickButton(m_buttonBox, RobotMap.kTrapShooterVelocityControl);
  JoystickButton m_centerSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kCenterSpeakerShooterVelocityControl);
  JoystickButton m_rightSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kRightSpeakerShooterVelocityControl);
  JoystickButton m_leftSpeakerShootButton =new JoystickButton(m_buttonBox, RobotMap.kLeftSpeakerShooterVelocityControl);
  JoystickButton m_reverseIntakeButton = new JoystickButton(m_buttonBox, RobotMap.kReverseIntake);
  JoystickButton m_fireNoteButton = new JoystickButton(m_buttonBox, RobotMap.kFireNote);
  JoystickButton m_setAmpVelocityButton = new JoystickButton(m_buttonBox, RobotMap.kAmpShootVelocity);


  private final DigitalOutput m_beastEyes = new DigitalOutput(RobotMap.kbeastEyesDigitalOutput);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    m_driveXConditioning.setDeadband(0.15);
    m_driveXConditioning.setExponent(kDriveXExponent);
    m_driveYConditioning.setDeadband(0.15);
    m_driveYConditioning.setExponent(kDriveYExponent);
    m_turnConditioning.setDeadband(0.2);
    m_turnConditioning.setExponent(1.4);

    // Autonomous set up
    Autos.registerPathPlannerNamedCommands(this);
    m_autoChooser = Autos.sendableChooserFullPathPlanner(this);
    // m_autoChooser = Autos.sendableChooser(this);
    SmartDashboard.putData("Auto/Routine", m_autoChooser);
    // m_angleStartChooser = Autos.sendableChooserAngleStart(this);
    // SmartDashboard.putData("Auto/StartLocation", m_angleStartChooser);

    // Configure the trigger bindings
    configureBindings();

    // Smart Dashboard
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

    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
          () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
          () -> m_robot.isTeleopEnabled()));
    m_lockWheeButton.whileTrue(new LockWheelsCommand(m_driveSubsystem));
    m_resetNavX.onTrue(new InstantCommand(() -> {m_driveSubsystem.resetNavX();}));

    m_trapAlignButton.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Trap)
      .andThen(new AlignWithTrapCommand(m_driveSubsystem, m_AprilTagPID)));
    m_ampAlignButton.whileTrue(new AlignWithAmpCommand(m_driveSubsystem, m_AprilTagPID));

    // All the Note delivery commands
    m_fireButtonRT.whileTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Generic)
      .alongWith(new WaitCommand(m_delayTimeForShooter).andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem))));

    m_centerSpeakerShootButton.onTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.CenterSpeaker)
      .alongWith(new RetractAmpAssistCommand(m_AmpAssistSubsystem)));
    m_leftSpeakerShootButton.onTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.LeftSpeaker)
      .alongWith(new RetractAmpAssistCommand(m_AmpAssistSubsystem)));
    m_rightSpeakerShootButton.onTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.RightSpeaker)
      .alongWith(new RetractAmpAssistCommand(m_AmpAssistSubsystem)));
    m_trapShootButton.onTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Trap));


    m_fireNoteButton.onTrue(new FeedNoteIntoShooterCommand(m_intakeSubsystem)
      .andThen(new WaitCommand (0.5),
        new InstantCommand(() -> {m_shooterSubsystem.stopShooterMotor(); }, m_shooterSubsystem)));

    // Pick up Note from Floor
    m_intakeButton.onTrue(new NoteIntakeFromFloorCommand(m_intakeSubsystem));
    m_reverseIntakeButton.whileTrue(new ReverseIntakeCommand(m_intakeSubsystem));

    // Amp Assist
    m_setAmpVelocityButton.onTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Amp));
    m_extendAmpAssistButton.onTrue(new ExtendAmpAssistCommand(m_AmpAssistSubsystem)
     .alongWith(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Amp)));
      // .andThen(new FeedNoteIntoShooterCommand(m_intakeSubsystem)));
    m_retractAmpAssistButton.onTrue(new RetractAmpAssistCommand(m_AmpAssistSubsystem));
    // m_extendAmpAssistButton.onTrue(new ShooterVelocityCommand(m_shooterSubsystem, ShooterLocation.Amp));
    //  m_extendAmpAssistButton.onTrue(new AmpDirectDrive(m_AmpAssistSubsystem, m_shooterSubsystem));

    // Climb
    m_extendClimbButton.onTrue(new ClimbExtendCommand(m_climbSubsystem));
    m_retractClimbButton.onTrue(new ClimbRetractCommand(m_climbSubsystem));
    m_latchClimbButton.onTrue(new ClimbLatchCommand(m_climbSubsystem));

    // Autonomous Trigger to stop finish floor intake
    // BEWARE!! Can stop drive auto! If has Intake Subsystem req
    new Trigger(m_intakeSubsystem::isNotePickedUp)
      .onTrue(new DetectNoteIntakeFromFloorCommandForAutonomous(m_intakeSubsystem)
          .andThen(new InstantCommand(() ->
              m_shooterSubsystem.controlShooterToVelocity(ShooterLocation.CenterSpeaker))));

    // Lights
    new Trigger(m_intakeSubsystem::isNotePickedUp)
      .onTrue(new InstantCommand(() -> m_bling.setBlingMessage(BlingMessage.Green)));
    new Trigger(m_intakeSubsystem::isNotePresent)
      .onFalse(new InstantCommand(() -> m_bling.setBlingMessage(getAllianceColor())));
    new Trigger(this::inLastTwentySeconds)
      .onTrue(new InstantCommand(() -> m_bling.setFlashState(true)));
  }

  public void smartDashboardInit() {
      SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
      m_driveSubsystem.smartDashboardInit();
      m_shooterSubsystem.smartDashboardInit();
      m_intakeSubsystem.smartDashboardInit();
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
          // m_AmpAssistSubsystem.smartDashboardUpdate();
          // m_climbSubsystem.smartDashboardUpdate();
          m_AprilTagPID.updateAprilTagSmartDashboard();
      }, 1, 0.303);
      // m_robot.addPeriodic(() -> {
      //     m_driveSubsystem.checkRelativeEncoderToAbsoluteEncoder();
      // }, 0.5, 0.107);
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
   // var startLocation = m_angleStartChooser.getSelected();

  //  switch (startLocation) {
    //  case BlueAmp:
   //     m_driveSubsystem.setStartAngle(60);
    //    break;
    //  case BlueSource:
     //   m_driveSubsystem.setStartAngle(-60);
     //   break;
    //  case RedAmp:
      //  m_driveSubsystem.setStartAngle(-60);
      //  break;
    //  case RedSource:
      //  m_driveSubsystem.setStartAngle(60);
     //   break;
     // case Center:
     // default:
      //  break;
   // }

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

  private BlingMessage getAllianceColor()
  {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red ? BlingMessage.Red : BlingMessage.Blue;
      }

      return BlingMessage.Blue;
  }

  private boolean inLastTwentySeconds()
  {
    return DriverStation.getMatchTime() <= 20.2;
  }

  public void setOnInitBlingState()
  {
    m_bling.setFlashState(false);
    m_bling.setBlingMessage(getAllianceColor());
  }

  public void setBeastEyes(boolean beastEyesOn)
  {
    m_beastEyes.set(beastEyesOn);
  }
}
