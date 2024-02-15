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

  private double m_movementSpeed = 0.25;

  private AnalogPotentiometer m_potentiometer;

  /** Creates a new AmpShooterSubsystem. */
  public AmpAssistSubsystem() 
  {
    // m_LeftAmpRampServo = new PWM(RobotMap.kLeftAmpServo); 
    m_RightAmpRampServo = new PWM(RobotMap.kRightAmpServo);

    m_potentiometer = new AnalogPotentiometer(RobotMap.kAmpStringPotAnalog);

    // m_LeftAmpRampServo.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
    m_RightAmpRampServo.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
 
    stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotor()
  {
    setSpeed(0.0);
  }

  public void startMoving(boolean extend)
  {
    if (extend)
      setSpeed(m_movementSpeed);
    else
      setSpeed(-m_movementSpeed);
  }

  //sets speed of ramp thing
  private void setSpeed(double speed)
  {
    // m_LeftAmpRampServo.setSpeed(-speed);
    m_RightAmpRampServo.setSpeed(speed);
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
      return getPosition() >= 5.5; // in inches
    }
    else
    {
      return getPosition() <= 0.25; // in inches
    }
  }

  public void smartDashboardInit()
  {
     SmartDashboard.putNumber(getName() + "/Amp Assist Speed", m_movementSpeed);
     SmartDashboard.putNumber(getName() + "/Amp Assist Position", getPosition());
     SmartDashboard.putBoolean(getName() + "/Drive At Speed", false);
     SmartDashboard.putBoolean(getName() + "/Stop", false);
    }

  public void smartDashboardUpdate()
  {
     SmartDashboard.putNumber(getName() + "/Target Speed", getPosition());
    
     if (SmartDashboard.getBoolean(getName() + "/Drive At Speed", false))
     {
        m_movementSpeed = SmartDashboard.getNumber(getName() + "/Target Speed", m_movementSpeed);
        setSpeed(m_movementSpeed);

        SmartDashboard.putBoolean(getName() + "/Drive At Speed", false);
     }
    
     if (SmartDashboard.getBoolean(getName() + "/Stop", false))
     {
        stopMotor();

        SmartDashboard.putBoolean(getName() + "/Stop", false);
     }
  }

}
