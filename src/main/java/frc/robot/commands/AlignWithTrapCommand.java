// Copyright (c) FIRST and other WPILib contributors.
// What? Who would hide an EasterEgg in here? 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Vision;

public class AlignWithTrapCommand extends Command {
  /** Creates a new AlignWithTrapCommand. */
  private DriveSubsystem m_driveSubsystem;
  private double m_lastRotation;
  private double m_lastDeltaX;
  private double m_lastDeltaZ;
  private double m_targetZ = 1.5;
  private double m_targetRotaion = 0;
  public AlignWithTrapCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);

    SmartDashboard.putNumber("AprilTag\\tx", 0);
    SmartDashboard.putNumber("AprilTag\\tz", 0);
    SmartDashboard.putNumber("AprilTag\\tx delta", 0);
    SmartDashboard.putNumber("AprilTag\\tz delta", 0);
    SmartDashboard.putNumber("AprilTag\\target Z", m_targetZ);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var tid = (int)LimelightHelpers.getFiducialID(Vision.kLimeLightName);
    if (tid == 15 || tid == 11)
      m_targetRotaion = -120;
    else if (tid == 16 || tid == 12)
      m_targetRotaion = 120;
    else
      m_targetRotaion = 0;

    m_targetZ = SmartDashboard.getNumber("AprilTag\\target Z", m_targetZ);
    m_lastRotation = 5;
    m_lastDeltaX = 5;
    m_lastDeltaZ = 5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_lastRotation = rotationTarget();

      //while using Limelight, turn off field-relative driving.
      m_driveSubsystem.drive(-xSpeed(), ySpeed(), m_lastRotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       m_driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_lastRotation < 0.5 && m_lastDeltaX < 0.1 && m_lastDeltaZ < 0.1;
  }

  private double xSpeed()
  {
    double kP = .1;
    var tx = LimelightHelpers.getTargetPose_RobotSpace(Vision.kLimeLightName)[0];
    double targetingForwardSpeed = m_lastDeltaX = tx * kP;
    SmartDashboard.putNumber("AprilTag\\tx", tx);
    SmartDashboard.putNumber("AprilTag\\tx delta", m_lastDeltaX);
    targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private double ySpeed()
  {
    double kP = .1;
    var tz = LimelightHelpers.getTargetPose_RobotSpace(Vision.kLimeLightName)[2];
    m_lastDeltaZ = m_targetZ - tz;
    SmartDashboard.putNumber("AprilTag\\tz", tz);
    SmartDashboard.putNumber("AprilTag\\tz delta", m_lastDeltaZ);
    double targetingForwardSpeed = m_lastDeltaZ * kP;
    targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private double rotationTarget()
  {
    double kP = .035;
    double targetingAngularVelocity = m_targetRotaion - m_driveSubsystem.getAngle().getDegrees()  * kP;
    targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
}
}
