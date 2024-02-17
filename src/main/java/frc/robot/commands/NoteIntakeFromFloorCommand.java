// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class NoteIntakeFromFloorCommand extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  private int m_ticks;

  /** Creates a new NoteIntakeFromFloorCommand. */
  public NoteIntakeFromFloorCommand(IntakeSubsystem intakeSubsystem)
  {
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ticks = 0;
    m_intakeSubsystem.intakeForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.isNotePresent())
    {
      m_ticks++;
      m_intakeSubsystem.reverseIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ticks >= 4;
  }
}
