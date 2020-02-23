/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;

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
public final class Constants {

  public static final class DriveConstants {
    public static final int motorTimeout = 30;// ms
    public static final int kLeftMotor1ID = 0;
    public static final int kLeftMotor2ID = 0;
    public static final int kRightMotor1ID = 0;
    public static final int kRightMotor2ID = 0;
    public static final double kEncoderDistancePerPulse = 0;// probably will do a bunch of math here
    public static final boolean kGyroReversed = false;
    public static final double kTrackwidthMeters = Units.inchesToMeters(20);

  }

  public static final class ControllerConst {
    public static final int leftSideAxisId = 0;
    public static final int rightSideAxisId = 1;
    public static final int curveDriveButton = 0;
    
  }

  public static final class AutoConstants {
    // autonomous
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        DriveConstants.kTrackwidthMeters);
  }

  // public static final int kPIDLoopIdx = 0;
  public static final double rawAxisMaxValue = 1;// do not touch

  public static final class LimelightMounting {
    public static final double LIMELIGHT_MOUNT_HEIGHT = 23.0; //inches
    public static final double LIMELIGHT_MOUNT_ANGLE = 15.0; //degrees
  }


}
