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

import com.ctre.phoenix.motorcontrol.ControlMode;
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

private final I2C.Port i2cPort = I2C.Port.kOnboard;
private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
private double IR = m_colorSensor.getIR();

private final SenseColor colorSense = new SenseColor();
private static final int kCanID = 1;
private static final MotorType kMotorType = MotorType.kBrushless;
private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
private static final int kCPR = 8192;

private CANSparkMax m_motor;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

private final CANSparkMax SpinnerMotor = new CANSparkMax(kSpinnerPort, MotorType.kBrushless);
private final CANPIDController spinController = SpinnerMotor.getPIDController();


public Spinner() {
  spinController.setP(shooterLeftPID.Kp);
  spinController.setI(shooterLeftPID.Ki);
  spinController.setD(shooterLeftPID.Kd); 
}

public double getRawColor() {
  return IR;
}
public void spin(){



  //spinController.setReference(10, ControlMode.Position);  
  if(colorSense.getIsBlue()){
    SpinnerMotor.set(.3);

  private final SenseColor colorSense = new SenseColor();

  private final CANSparkMax SpinnerMotor = new CANSparkMax(kSpinnerPort, MotorType.kBrushless);
  private final CANPIDController spinController = SpinnerMotor.getPIDController();


  public Spinner() {
    spinController.setP(shooterLeftPID.Kp);
    spinController.setI(shooterLeftPID.Ki);
    spinController.setD(shooterLeftPID.Kd); 
  }


  public void measuredSpin(double rotations) {
    spinController.setReference(rotations, ControlType.kPosition);  

  }




  @Override
  public void periodic() {

  }
}
}
