// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterVelocityCommand extends Command {
  private ShooterSubsystem m_shooter;
  // private IntakeSubsystem m_intake;
  private int m_ticks;
  private ShooterSubsystem.ShooterLocation m_location;

  /** Creates a new ShooterVelocityComma`nd. */
  public ShooterVelocityCommand(
    ShooterSubsystem shooterSubsystem,
    ShooterSubsystem.ShooterLocation location)
  {
    m_shooter = shooterSubsystem;
    m_location = location;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.controlShooterToVelocity(m_location);
    m_ticks = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (++m_ticks == 12)
    {
      // m_intake.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooterMotor();
    // m_intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
