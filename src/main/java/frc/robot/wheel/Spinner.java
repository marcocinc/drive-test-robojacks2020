/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.platform.DeviceType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Spinner extends SubsystemBase {

private final SenseColor colorSense = new SenseColor();
private static final int kCanID = 1;
private static final MotorType kMotorType = MotorType.kBrushless;
private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
private static final int kCPR = 8192;

private CANSparkMax m_motor;
private CANPIDController m_pidController;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


private CANEncoder m_alternateEncoder;

CANSparkMax SpinnerMotor = new CANSparkMax(kSpinnerPort, MotorType.kBrushless);


public Spinner() {
  
}


public void spin(){

  m_motor = new CANSparkMax(kCanID, kMotorType);
  m_alternateEncoder = m_motor.getAlternateEncoder(kAltEncType, kCPR);

  m_motor.restoreFactoryDefaults();
  m_pidController = m_motor.getPIDController();
  m_pidController.setFeedbackDevice(m_alternateEncoder);


  kP = 0.1; 
  kI = 1e-4;
  kD = 1; 
  kIz = 0; 
  kFF = 0; 
  kMaxOutput = 1; 
  kMinOutput = -1;

  m_pidController.setP(kP);
  m_pidController.setI(kI);
  m_pidController.setD(kD);
  m_pidController.setIZone(kIz);
  m_pidController.setFF(kFF);
  m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  SmartDashboard.putNumber("P Gain", kP);
  SmartDashboard.putNumber("I Gain", kI);
  SmartDashboard.putNumber("D Gain", kD);
  SmartDashboard.putNumber("I Zone", kIz);
  SmartDashboard.putNumber("Feed Forward", kFF);
  SmartDashboard.putNumber("Max Output", kMaxOutput);
  SmartDashboard.putNumber("Min Output", kMinOutput);
  SmartDashboard.putNumber("Set Rotations", 0);


}


@Override
public void periodic() {

  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);
  double rotations = SmartDashboard.getNumber("Set Rotations", 0);

if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    m_pidController.setReference(rotations, ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_alternateEncoder.getPosition());

}
}
