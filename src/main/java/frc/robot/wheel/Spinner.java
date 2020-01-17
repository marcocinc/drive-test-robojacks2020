/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import static frc.robot.Constants.*;

public class Spinner extends SubsystemBase {

private final SenseColor colorSense = new SenseColor();

  WPI_TalonSRX SpinnerMotor = new WPI_TalonSRX(kSpinnerPort);
 
  private double counter = 0; 

  public Spinner() {
    
  }


  public void Spin(){

    if (i<20 && colorSense.getIsBlue()){

      
    }

  }


  @Override
  public void periodic() {
  
  }
}
