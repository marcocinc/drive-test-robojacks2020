/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class TalonDrivetrain extends SubsystemBase {

    WPI_TalonSRX LFrontWheel = new WPI_TalonSRX(kLeftFrontPort);
    WPI_TalonSRX RFrontWheel = new WPI_TalonSRX(kRightFrontPort);

    WPI_TalonSRX LRearWheel = new WPI_TalonSRX(kLeftRearPort);
    WPI_TalonSRX RRearWheel = new WPI_TalonSRX(kRightRearPort);

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ktrackWidthMeters);
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    SimpleMotorFeedforward feedforward 
    = new SimpleMotorFeedforward(DriveFeedforward.ks, DriveFeedforward.kv, DriveFeedforward.ka);

    PIDController leftDrivePID 
    = new PIDController(leftDrive.Kp, leftDrive.Ki, leftDrive.Kd);

    PIDController rightDrivePID
    = new PIDController(rightDrive.Kp, rightDrive.Ki, rightDrive.Kd);

    Pose2d pose = new Pose2d();

    public TalonDrivetrain() {
        LRearWheel.follow(LFrontWheel);
        RRearWheel.follow(RFrontWheel);

        LFrontWheel.setInverted(kLeftInverted);
        RRearWheel.setInverted(kRightInverted);

        LFrontWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        LFrontWheel.setSelectedSensorPosition(0);
        LFrontWheel.setSensorPhase(kLeftEncoderPhase);

        RFrontWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RFrontWheel.setSelectedSensorPosition(0);
        RFrontWheel.setSensorPhase(kRightEncoderPhase);
    
        gyro.reset();
    }

    public void tankDrive(double leftOutput, double rightOutput) {
        LFrontWheel.set(leftOutput);
        RFrontWheel.set(rightOutput);
    }

    public void setOutputVolts(double leftVolts, double rightVolts) {
        LFrontWheel.setVoltage(leftVolts);
        RFrontWheel.setVoltage(rightVolts);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
      }
    
      public Pose2d getPose() {
        return pose;
      }
    
      public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
      }
    
      public PIDController getLeftDrivePID() {
        return leftDrivePID;
      }
    
      public PIDController getRightDrivePID() {
        return rightDrivePID;
      }
    
    public double getLeftDistanceMeters() {
        return LFrontWheel.getSelectedSensorPosition() / kTicksPerRev * 2 * Math.PI * kWheelRadiusMeters;
    }

    public double getRightDistanceMeters() {
        return RFrontWheel.getSelectedSensorPosition() / kTicksPerRev * 2 * Math.PI * kWheelRadiusMeters;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            LFrontWheel.getSelectedSensorVelocity() / kGearRatio * 2 * Math.PI * kWheelRadiusMeters / 60,
            RFrontWheel.getSelectedSensorVelocity() / kGearRatio * 2 * Math.PI * kWheelRadiusMeters / 60
        );
      }
    
   /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
   @Override
   public void periodic() {
        pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
   }
}
