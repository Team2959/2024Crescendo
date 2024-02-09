// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new ReverseIntakeCommand. */
  public ReverseIntakeCommand(IntakeSubsystem IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem);
    m_IntakeSubsystem = IntakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.reverseIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
