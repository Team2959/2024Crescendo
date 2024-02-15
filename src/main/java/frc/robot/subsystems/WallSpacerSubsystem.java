// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class WallSpacerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_wallSpacerMotor = new WPI_TalonSRX(RobotMap.kWallSpacerVictorSpMotor);
  private final DigitalInput m_wallSpacerExtended = new DigitalInput(RobotMap.kWallSpacerExtendedDigitalInput);
  private final DigitalInput m_wallSpacerRetracted = new DigitalInput(RobotMap.kWallSpacerRetractedDigitalInput);

  double m_wallSpacerSpeed = 0.75;

  /** Creates a new WallSpacerSubsystem. */
  public WallSpacerSubsystem() {
    m_wallSpacerMotor.setInverted(true);
  }
  
  @Override
  public void periodic() {
  }
 
  public void smartDashboardInit(){
    SmartDashboard.putNumber(getName() + "/Target Speed", m_wallSpacerSpeed);
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Extended", isWallSpacerExtended());
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Retracted", isWallSpacerRetracted());
    SmartDashboard.putBoolean(getName() + "/Move Wall Spacer", false);
    SmartDashboard.putBoolean(getName() + "/Stop", false);
  }

  public void smartDashboardUpdate(){
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Extended", isWallSpacerExtended());
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Retracted", isWallSpacerRetracted());
  
    if (SmartDashboard.getBoolean(getName() + "/Move Wall Spacer", false))
    {
       m_wallSpacerSpeed = SmartDashboard.getNumber(getName() + "/Target Speed", m_wallSpacerSpeed);
       m_wallSpacerMotor.set(m_wallSpacerSpeed);

       SmartDashboard.putBoolean(getName() + "/Move Wall Spacer", false);
    }

    if (SmartDashboard.getBoolean(getName() + "/Stop", false))
    {
       stopWallSpacer();

       SmartDashboard.putBoolean(getName() + "/Stop", false);
    }
  }

  public void extendWallSpacer()
  {
    m_wallSpacerMotor.set(m_wallSpacerSpeed);
  }

  public void retractWallSpacer()
  {
    m_wallSpacerMotor.set(-m_wallSpacerSpeed);
  }

  public void stopWallSpacer()
  {
    m_wallSpacerMotor.set(0.0);
  }

  public boolean isWallSpacerExtended()
  {
    return m_wallSpacerExtended.get(); 
  }

  public boolean isWallSpacerRetracted()
  {
    return m_wallSpacerRetracted.get();
  }
}
