/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  CANSparkMax fRightMotor = new CANSparkMax(DriveConstants.DRV_RIGHT_FRONT, MotorType.kBrushless);
  CANSparkMax bRightMotor = new CANSparkMax(DriveConstants.DRV_RIGHT_BACK, MotorType.kBrushless);
  CANSparkMax fLeftMotor = new CANSparkMax(DriveConstants.DRV_LEFT_FRONT, MotorType.kBrushless);
  CANSparkMax bLeftMotor = new CANSparkMax(DriveConstants.DRV_LEFT_BACK, MotorType.kBrushless);

  //Drive groups
  private SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(fRightMotor, bRightMotor);
  private SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(fLeftMotor, bLeftMotor);

  //Diff Drive
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
  //                 DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
  //                 DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final AnalogGyro m_gyro = new AnalogGyro(1);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(getRotation2d(), fLeftMotor.getEncoder().getPosition(),
                      fRightMotor.getEncoder().getPosition());
  }

  public Rotation2d getRotation2d() {
    // Converts angle of gyro in degrees to radians and returns Rotation2d object
    return new Rotation2d(m_gyro.getAngle()*(Math.PI/180));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(fLeftMotor.getEncoder().getVelocity(), fRightMotor.getEncoder().getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    fLeftMotor.getEncoder().setPosition(0);
    fRightMotor.getEncoder().setPosition(0);
    bLeftMotor.getEncoder().setPosition(0);
    bRightMotor.getEncoder().setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (fLeftMotor.getEncoder().getPosition() + fRightMotor.getEncoder().getPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}