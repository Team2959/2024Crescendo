// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbLatchCommand extends Command {
private ClimbSubsystem m_climbSubsystem;
  /** Creates a new ClimbLatchCommand. */
  public ClimbLatchCommand(ClimbSubsystem climbSubsystem) {
    m_climbSubsystem = climbSubsystem;
    // Use addRequirements() here to declare subsystem dependenc
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.latchClimbArms();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      m_climbSubsystem.stopAtCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climbSubsystem.isAtTargetPosition();
  }
}
