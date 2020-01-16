/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.turret;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GoalMover extends SubsystemBase {
  /**
   * create new solenoides
   */
  Solenoid lowSolenoid = new Solenoid(Constants.lowSolenoidPort);
  Solenoid highSolenoid= new Solenoid(Constants.highSolenoidPort);


  // state keeps track of how high is the cannon
  public enum GoalState {
    COLLECTING, HIGH, LOW;
  }
  public GoalState state = GoalState.HIGH;

  /**
   * switch from a shooting position to a collecting position and vice versa.
   */ 
  public void SwapCollecting(){
    switch(state){
      case COLLECTING:
        lowGoal();
      
      default:
        CollectGoal();
    }
    
  }
  /**
   * switch between a high shooting position and a low shooting position 
   */


  public void SwapHeight(){
    switch(state){
      case HIGH:
        lowGoal();
      case LOW:
        highGoal();
      default:
        lowGoal();

    }
  
  }



  public void CollectGoal(){
    
    state = GoalState.COLLECTING;

    lowSolenoid.set(false);
    highSolenoid.set(false);
  }

  public void lowGoal(){

    state = GoalState.LOW;
    
    lowSolenoid.set(true);
    highSolenoid.set(false);
  }

  public void highGoal(){

    state = GoalState.HIGH;
    
    lowSolenoid.set(false);
    highSolenoid.set(true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
