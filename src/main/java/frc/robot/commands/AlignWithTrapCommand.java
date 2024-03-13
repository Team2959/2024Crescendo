// Copyright (c) FIRST and other WPILib contributors.
// What? Who would hide an EasterEgg in here? 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class AlignWithTrapCommand extends Command {
  /** Creates a new AlignWithTrapCommand. */
  private DriveSubsystem m_driveSubsystem;
  private double m_lastRotation;
  private double m_lastDriveSpeed;
  private double m_targetTy = 27;
  public AlignWithTrapCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);

    SmartDashboard.putNumber("AprilTag\tx", 0);
    SmartDashboard.putNumber("AprilTag\ty", 0);
    SmartDashboard.putNumber("AprilTag\ty delta", 0);
    SmartDashboard.putNumber("AprilTag\target ty", m_targetTy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetTy = SmartDashboard.getNumber("AprilTag\target ty", 27);
    m_lastRotation = 5;
    m_lastDriveSpeed = 5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_lastRotation = Vision.limelight_aim_proportional();

      m_lastDriveSpeed = Vision.limelight_range_proportional(m_targetTy);

      //while using Limelight, turn off field-relative driving.
      m_driveSubsystem.drive(-m_lastDriveSpeed, 0, m_lastRotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       m_driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_lastRotation < 0.5 && m_lastDriveSpeed < 0.5;
  }  
}
