/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */
  
   WPI_TalonSRX motor = new WPI_TalonSRX(0);
   
   public Arm() {

  }

  public void reach(){
  
  motor.set(0.75);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


