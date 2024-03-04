// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AmpAssistSubsystem extends SubsystemBase {
  private PWM m_LeftAmpRampServo; 
  private PWM m_RightAmpRampServo;

  private AnalogPotentiometer m_potentiometer;

  /** Creates a new AmpShooterSubsystem. */
  public AmpAssistSubsystem() 
  {
    m_LeftAmpRampServo = new PWM(RobotMap.kLeftAmpServo); 
    m_RightAmpRampServo = new PWM(RobotMap.kRightAmpServo);

    m_potentiometer = new AnalogPotentiometer(RobotMap.kAmpStringPotAnalog);

    m_LeftAmpRampServo.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
    m_RightAmpRampServo.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
 
    stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotor()
  {
    m_LeftAmpRampServo.setDisabled();
    m_RightAmpRampServo.setDisabled();
  }

  public void startMoving(boolean extend)
  {
    if (extend)
    {
      m_LeftAmpRampServo.setPulseTimeMicroseconds(1000);
      m_RightAmpRampServo.setPulseTimeMicroseconds(2000);
    }
    else
    {
      m_LeftAmpRampServo.setPulseTimeMicroseconds(2000);
      m_RightAmpRampServo.setPulseTimeMicroseconds(1000);
    }
  }

  private double getPosition()
  {
    // ToDo: convert to inches!!
    return m_potentiometer.get();
  }

  public boolean AtPosition(boolean extended)
  {
    if (extended)
    {
      return getPosition() >= 0.7;
    }
    else
    {
      return getPosition() <= 0.15;
    }
  }

  public void smartDashboardInit()
  {
     SmartDashboard.putNumber(getName() + "/Position", getPosition());
    //  SmartDashboard.putNumber(getName() + "/Target Speed", 1500);
    //  SmartDashboard.putBoolean(getName() + "/Drive At Speed", false);
    //  SmartDashboard.putBoolean(getName() + "/Stop", false);
  }

  public void smartDashboardUpdate()
  {
     SmartDashboard.putNumber(getName() + "/Position", getPosition());
    
  //   if (SmartDashboard.getBoolean(getName() + "/Drive At Speed", false))
  //   {
  //     double movementSpeed = SmartDashboard.getNumber(getName() + "/Target Speed", 1500);
  //     m_LeftAmpRampServo.setPulseTimeMicroseconds((int)movementSpeed);
  //     m_RightAmpRampServo.setPulseTimeMicroseconds((int)movementSpeed);

  //     SmartDashboard.putBoolean(getName() + "/Drive At Speed", false);
  //   }

  //   if (SmartDashboard.getBoolean(getName() + "/Stop", false))
  //   {
  //       stopMotor();

  //       SmartDashboard.putBoolean(getName() + "/Stop", false);
  //   }
  }
}
