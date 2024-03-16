// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AmpAssistSubsystem extends SubsystemBase {
  private CANSparkMax m_AmpRampNEO; 
  private SparkRelativeEncoder m_AmpRampEncoder;
  private SparkPIDController m_ampPidController;
  
  private double m_lastTarget = 0;
  private double m_extendDistance = 0.7; //no idea what this is
  private double m_retractDistance = 0.15; //no idea what this is

  private static final double kAmpP = 0.15;   //guess
  private static final double kAmpI = 0.0;   //guess
  private static final double kAmpD = 0.0;   //guess
  private static final double kAmpFF = 0;   //guess
  private static final double kAmpIZone = 0;   //guess

  /** Creates a new AmpShooterSubsystem. */
  public AmpAssistSubsystem() 
  {
    m_AmpRampNEO = new CANSparkMax(RobotMap.kAmpNEO, CANSparkMax.MotorType.kBrushless);

    m_AmpRampNEO.restoreFactoryDefaults();

    m_AmpRampNEO.setIdleMode(IdleMode.kBrake);
 
    m_AmpRampEncoder = (SparkRelativeEncoder)m_AmpRampNEO.getEncoder();

    stopMotor();

    m_ampPidController = m_AmpRampNEO.getPIDController();
    m_ampPidController.setP(kAmpP);
    m_ampPidController.setI(kAmpI);
    m_ampPidController.setD(kAmpD);
    m_ampPidController.setFF(kAmpFF);
    m_ampPidController.setIZone(kAmpIZone);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotor()
  {
    m_AmpRampNEO.set(0);
  }

  public double getAmpPosition()
  {
    return m_AmpRampEncoder.getPosition();
  }

  public boolean isAtPosition()
  {
    return Math.abs(m_lastTarget - getAmpPosition()) < 0.5;
  }

  public void smartDashboardInit()
  {
     SmartDashboard.putNumber(getName() + "/Position", getAmpPosition());
    //  SmartDashboard.putNumber(getName() + "/Target Speed", 1500);
    //  SmartDashboard.putBoolean(getName() + "/Drive At Speed", false);
    //  SmartDashboard.putBoolean(getName() + "/Stop", false);
  }

  public void smartDashboardUpdate()
  {
     SmartDashboard.putNumber(getName() + "/Position", getAmpPosition());
    
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

  private void setTargetPosition(double position)
  {
    m_ampPidController.setReference(position, ControlType.kPosition);
    m_lastTarget = position;
  }

  public void extendAmpRamp()
  {
    setTargetPosition(m_extendDistance); 
  }

  public void retractAmpRamp()
  {
    setTargetPosition(m_retractDistance);
  }
}
