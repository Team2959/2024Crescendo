// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpAssistSubsystem;

public class ExtendAmpAssistCommand extends Command {
  private AmpAssistSubsystem m_AmpAssistSubsystem; 
  /** Creates a new ExtendAmpAssistCommand. */
  public ExtendAmpAssistCommand(AmpAssistSubsystem ampAssistSubsystem) {
    m_AmpAssistSubsystem = ampAssistSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_AmpAssistSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AmpAssistSubsystem.startMoving(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AmpAssistSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_AmpAssistSubsystem.AtPosition(true);
  }
}
