// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class WallSpacerSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_wallSpacerMotor = new WPI_VictorSPX(RobotMap.kWallSpacerVictorSpMotor);
  private final DigitalInput m_wallSpacerExtended = new DigitalInput(RobotMap.kWallSpacerExtendedDigitalInput);
  private final DigitalInput m_wallSpacerRetracted = new DigitalInput(RobotMap.kWallSpacerRetractedDigitalInput);

  double m_wallSpacerSpeed = 0.25; //guess

  /** Creates a new WallSpacerSubsystem. */
  public WallSpacerSubsystem() {
  }
  
  @Override
  public void periodic() {
  }
 
  public void smartDashboardInit(){
    SmartDashboard.getNumber(getName() + "/Wall Spacer Target Speed",m_wallSpacerSpeed);
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Extended", isWallSpacerExtended());
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Retracted", isWallSpacerRetracted());
  }

  public void smartDashboardUpdate(){
    m_wallSpacerSpeed = SmartDashboard.getNumber(getName() + "/Wall Spacer Target Speed", m_wallSpacerSpeed);
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Extended", isWallSpacerExtended());
    SmartDashboard.putBoolean(getName() + "/Wall Spacer Retracted", isWallSpacerRetracted());
  }

  public void extendWallSpacer()
  {
    m_wallSpacerMotor.set(m_wallSpacerSpeed); //not sure whether positive is extending or retracting
  }

  public void retractWallSpacer()
  {
    m_wallSpacerMotor.set(-m_wallSpacerSpeed); //not sure whether negative is extending or retracting
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
