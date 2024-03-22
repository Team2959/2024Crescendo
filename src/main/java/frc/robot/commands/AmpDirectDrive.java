// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpAssistSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterLocation;

public class AmpDirectDrive extends Command {
  AmpAssistSubsystem m_amp;
  ShooterSubsystem m_shooter;
  /** Creates a new AmpDirectDrive. */
  public AmpDirectDrive(AmpAssistSubsystem amp, ShooterSubsystem shooter)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp, shooter);
    m_amp = amp;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.controlShooterToVelocity(ShooterLocation.Amp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_amp.directDrivePower();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_amp.extendAmpRamp(AmpAssistSubsystem.kDefaultSlot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_amp.getAmpPosition() < m_amp.m_extendDistance;
  }
}
