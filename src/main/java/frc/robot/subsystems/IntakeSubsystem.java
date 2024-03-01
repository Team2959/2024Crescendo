// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(RobotMap.kIntakeVictorSpMotor);
  private final DigitalInput m_noteDetect = new DigitalInput(RobotMap.kNoteDetectorDigitalInput);

  double m_intakeSpeed = 1.0;
  double m_reverseIntakeSpeed = -0.25;
  boolean m_isPickingUpNote;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(getName() + "/Note Detect", isNotePresent());
  }

  public void smartDashboardInit()
  {
    SmartDashboard.putNumber(getName() + "/Target Speed", m_intakeSpeed);
    SmartDashboard.putNumber(getName() + "/Reverse Speed", m_reverseIntakeSpeed);
  }

  public void smartDashboardUpdate()
  {
    m_intakeSpeed = SmartDashboard.getNumber(getName() + "/Target Speed", m_intakeSpeed);
    m_reverseIntakeSpeed = SmartDashboard.getNumber(getName() + "/Reverse Speed", m_reverseIntakeSpeed);
  }

  public void intakeForward() 
  {
    m_intakeMotor.set(m_intakeSpeed);
  }

  public void reverseIntake() {
    m_intakeMotor.set(m_reverseIntakeSpeed);
    m_isPickingUpNote = false;
  }

  public void stopMotor() {
    m_intakeMotor.set(0);
    m_isPickingUpNote = false;
  }

  public boolean isNotePresent()
  {
    return m_noteDetect.get() == false;
  }

  public boolean isNotePickedUp()
  {
    return m_isPickingUpNote && isNotePresent();
  }

  public void setPickingUpNote() {
      m_isPickingUpNote = true;
  }
}
