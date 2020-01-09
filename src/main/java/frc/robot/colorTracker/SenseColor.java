/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.colorTracker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color; 

public class SenseColor extends SubsystemBase {
  /**
   * Creates a new SenseColor.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  
    
    
  public SenseColor() {
    
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final Color detectedColor = m_colorSensor.getColor();
    final double IR = m_colorSensor.getIR();
  
    SmartDashboard.putNumber("Red", detectedColor.red);
  
    SmartDashboard.putNumber("Green", detectedColor.green);
  
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  
    SmartDashboard.putNumber("IR", IR);
  
    final int proximity = m_colorSensor.getProximity();
  
  
  
    SmartDashboard.putNumber("Proximity", proximity);
  }
}
