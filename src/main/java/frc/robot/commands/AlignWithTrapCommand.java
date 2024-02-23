// Copyright (c) FIRST and other WPILib contributors.
// What? Who would hide an EasterEgg in here? 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class AlignWithTrapCommand extends Command {
  /** Creates a new AlignWithTrapCommand. */
  private DriveSubsystem m_driveSubsystem;
  private Vision m_vision;
  public AlignWithTrapCommand(DriveSubsystem driveSubsystem, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_vision = vision;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var rot_limelight = m_vision.limelight_aim_proportional();

    var forward_limelight = m_vision.limelight_range_proportional();

      //while using Limelight, turn off field-relative driving.
      m_driveSubsystem.drive(-forward_limelight, 0, rot_limelight, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       m_driveSubsystem.drive(0, 0, 0, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }

  
}
