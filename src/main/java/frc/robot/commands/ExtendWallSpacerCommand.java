// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WallSpacerSubsystem;

public class ExtendWallSpacerCommand extends Command {
  /** Creates a new ExtendWallSpacerCommand. */
  private WallSpacerSubsystem m_WallSpacerSubsystem;

  public ExtendWallSpacerCommand(WallSpacerSubsystem wallSpacerSubsystem) {
    m_WallSpacerSubsystem = wallSpacerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_WallSpacerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_WallSpacerSubsystem.extendWallSpacer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WallSpacerSubsystem.stopWallSpacer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_WallSpacerSubsystem.isWallSpacerExtended() || m_WallSpacerSubsystem.isWallSpacerRetracted();
  }
}
