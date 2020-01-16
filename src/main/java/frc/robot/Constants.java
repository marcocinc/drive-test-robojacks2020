/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Port Numbers
	public static final int kLeftFrontPort = 1;
	public static final int kRightFrontPort = 2;
	public static final int kLeftRearPort = 3;
  public static final int kRightRearPort = 4;

  public static final int kControllerPort = 0;

  public static final int kLeftShooterWheelPort = 5;
	public static final int kRightShooterWheelPort = 6;
    
  // Motor Phase
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = true;

  // Encoder Sensor Phase
  public static final boolean kLeftEncoderPhase = false;
  public static final boolean kRightEncoderPhase = false;

  // Robot Measurements
  public static final int kTicksPerRev = 4096;
  public static final double ktrackWidthMeters = Units.inchesToMeters(10);
  public static final double kGearRatio = 7.29;
  public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);
	public static final double MaxSafeVelocityMeters = Units.feetToMeters(2);
	public static final double MaxSafeAccelerationMeters = Units.feetToMeters(2);
	public static final double targetToCameraHeight = 5;
  public static final double cameraAngle = 0;
  
  // Color Value Bounds
	public static final int blueLowerBound = 12;
  public static final int blueUpperBound = 13;

  public static final int redLowerBound = 9;
  public static final int redUpperBound = 10;

  public static final int greenLowerBound = 7;
  public static final int greenUpperBound = 8;

  public static final int yellowLowerBound = 11;
	public static final int yellowUpperBound = 12;
  
  public static class DriveFeedforward {
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;
  }
    
  public static class leftDrive {
    public static final double Kp = 0.1;
    public static final double Ki = 0;
    public static final double Kd = 0;
  }

  public static class rightDrive {
    public static final double Kp = 0.1;
    public static final double Ki = 0;
    public static final double Kd = 0;
  }

  public static class Ramsete {
    public static final double kb = 2;
    public static final double kzeta = 0.7;
  }
  
}