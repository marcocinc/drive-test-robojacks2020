/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private CANSparkMax leftLauncher = new CANSparkMax(kLeftShooterWheelPort, MotorType.kBrushless);
  private CANSparkMax rightLauncher = new CANSparkMax(kRightShooterWheelPort, MotorType.kBrushless);

  private CANPIDController leftController = new CANPIDController(leftLauncher);
  private CANPIDController rightController = new CANPIDController(rightLauncher);

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    leftController.setP(shooterLeftPID.Kp);
    leftController.setI(shooterLeftPID.Ki);
    leftController.setD(shooterLeftPID.Kd);    

    rightController.setP(shooterRightPID.Kp);
    rightController.setI(shooterRightPID.Ki);
    rightController.setD(shooterRightPID.Kd);
  }

  public void setVelocity(double rpm) {
    leftController.setReference(rpm, ControlType.kVelocity);
    rightController.setReference(-rpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
