/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class DriveConstants {

    // Feedforward/Feedback Gains
    /* Feedforward and feedback gains do not, in general, transfer across robots.
    Do not use the gains from the tutorial for your own robot. */
    public static final double ksVolts = 0.224;
    public static final double kvVoltSecondsPerMeter = 1.24;
    public static final double kaVoltSecondsSquaredPerMeter = 0.207;

    public static final double kPDriveVel = 0.00278;


    // Differential Drive Kinematics
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);


    // Max Trajectory Velocity/Acceleration
    /* The maximum velocity value should be set somewhat below the nominal
    free-speed of the robot. Due to the later use of the DifferentialDriveVoltageConstraint,
    the maximum acceleration value is not extremely crucial. */
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;


    // Ramsete Parameters
    /* The values shown below should work well for most robots, provided distances have
    been correctly measured in meters. */
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final int DRV_RIGHT_FRONT = 10;
    public static final int DRV_RIGHT_BACK = 11;
    public static final int DRV_LEFT_FRONT = 12;
    public static final int DRV_LEFT_BACK = 13;
}
