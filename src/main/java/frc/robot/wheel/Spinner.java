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

  public void toSelectedColor(String Color) {
    
  }


  @Override
  public void periodic() {

  }

}

