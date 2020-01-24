/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;



public class Arm extends SubsystemBase {
  private Solenoid arm = new Solenoid(compressorModule, armPort);

  public enum ArmState {
    REACH, PULL;
  }

  // Keeps track of how high the shooter is
  private ArmState state = ArmState.PULL;

  public void reach(){
    arm.set(true);
    state = ArmState.REACH;
  }

  public void pull(){
    arm.set(false);
    state = ArmState.PULL;
  }

  public void switchState() {
    switch(state) {
      case PULL:
        reach();
      case REACH:
        pull();
      default:
        pull();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


