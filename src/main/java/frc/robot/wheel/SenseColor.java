/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color; 

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
public class SenseColor extends SubsystemBase {
  /**
   * Creates a new SenseColor.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int proximity = m_colorSensor.getProximity();
  private double IR = m_colorSensor.getIR();
  private Color detectedColor = m_colorSensor.getColor();
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
 
  public String colorString;
  public ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

<<<<<<< HEAD

  boolean isBlue = getRawColor() >= blueLowerBound && getRawColor() <= blueUpperBound;
  boolean isRed = getRawColor() >= redLowerBound && getRawColor() <= redUpperBound; 
  boolean isYellow = getRawColor() >= yellowLowerBound && getRawColor() <= yellowUpperBound; 
  boolean isGreen = getRawColor() >= greenLowerBound && getRawColor() <= yellowUpperBound; 

=======
  private boolean isBlue = getRawColor() >= blueLowerBound && getRawColor() <= blueUpperBound;
  
>>>>>>> 88be0edda08ac7bdd0f6a5ef79198d345573ff19
  public boolean getIsBlue(){
    isBlue = getRawColor() >= blueLowerBound && getRawColor() <= blueUpperBound;

    return isBlue; 
  }

<<<<<<< HEAD
 public boolean getIsRed(){

  isRed = getRawColor() >= redLowerBound && getRawColor() <= redUpperBound; 

  return isRed; 

 }

 public boolean getIsYellow(){

  isYellow = getRawColor() >= yellowLowerBound && getRawColor() <= yellowUpperBound; 

  return isYellow; 

 }

 public boolean getIsGreen(){

  isGreen = getRawColor() >= greenLowerBound && getRawColor() <= yellowUpperBound; 

  return isGreen; 

 }


=======
>>>>>>> 88be0edda08ac7bdd0f6a5ef79198d345573ff19
  public Color getColor() {
    return detectedColor;
  }

  public double getRawColor() {
    return IR;
  }

  public int getProximity() {
    return proximity;
  }

  public String getColorString() {
    
    if (getRawColor() >= blueLowerBound && getRawColor() <= blueUpperBound) {
      return  colorString = "Blue";

    } else if (getRawColor() >= redLowerBound && getRawColor() <= redUpperBound) {
      return colorString = "Red";

    } else if (getRawColor() >= greenLowerBound && getRawColor() <= greenUpperBound) {
      return colorString = "Green";

    } else if (getRawColor() >= yellowLowerBound && getRawColor() <= yellowUpperBound) {
      return colorString = "Yellow";

    } else {
      return colorString = "Error: Unknown";

    }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedColor = m_colorSensor.getColor();
    IR = m_colorSensor.getIR();
    proximity = m_colorSensor.getProximity();
    match = m_colorMatcher.matchClosestColor(detectedColor);
  }
}
