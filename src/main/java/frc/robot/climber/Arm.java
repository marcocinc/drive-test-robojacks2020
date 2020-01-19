/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;



public class Arm extends SubsystemBase {
  private WPI_TalonSRX arm = new WPI_TalonSRX(armPort);

  public void reach(){
    arm.set(armPercentSpeed);
  }

  public void pull(){
    arm.set(-armPercentSpeed);
  }

  public void move(double speed) {
    arm.set(speed);
  }

  public void stop() {
    arm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


