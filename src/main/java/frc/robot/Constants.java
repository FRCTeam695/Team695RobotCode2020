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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Utility.Tuple;
import frc.robot.subsystems.dashTab;
import frc.robot.subsystems.dashTab.box;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static
 * final). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  // public static final int kPIDLoopIdx = 0;
  public static final int leftYstick = 0;

  public static final class FixColorConst {
    public static final double finalRotationsThreshold = 4;// number of color wheel spins, better to keep it slightly
                                                           // lower I think
    public static final double motorWheelRadius = 2.5;
    public static final double colorWheelRadius = 41.5;
	public static final double kTurnP = 0;
	public static final double kTurnD = 0;
  }

  public static final class ColorConst {

    public static final int[] speeds = { -7, -5, -1, 0, 1, 5, 7 };// in rpm
    public final static Color Blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final static Color Green = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final static Color Red = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final static Color Yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);
	public static final int PIDLoopId = 0;
	public static final int timoutMs = 0;
	public static int motorID=0;
  }

  public static final class DriveConstants {
    public static final int MOTOR_TIMEOUT = 30;// ms
    public static final int LEFT_MOTOR1_ID = 11;
    public static final int LEFT_MOTOR2_ID = 12;
    public static final int RIGHT_MOTOR1_ID = 13;
    public static final int RIGHT_MOTOR_2_ID = 14;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static final double ENCODER_DISTANCE_PER_WHEEL_ROTATION = 17305.6*Math.PI*WHEEL_DIAMETER;// probably will do a bunch of math here
    public static final boolean IS_GYRO_REVERSED = false;
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.5);//19.5 inch between, each wheel is inch thick

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
    public static final double ksVolts = 0.165;
    public static final double kvVoltSecondsPerMeter = 1.91;
    public static final double kaVoltSecondsSquaredPerMeter = 0.125;
    public static final double kPDriveVel = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        DriveConstants.TRACK_WIDTH);
  }

  // public static final int kPIDLoopIdx = 0;
  public static final double rawAxisMaxValue = 1;// do not touch

  public static final class LimelightMounting {
    public static final double LIMELIGHT_MOUNT_HEIGHT = 23.0; //inches
    public static final double LIMELIGHT_MOUNT_ANGLE = 15.0; //degrees
  }

  public static final class AuxiliaryMotorIds {
    public static final int HOPPER_VICTOR_ID = 6;
    public static final int INTAKE_VICTOR_ID = 5;
    public static final int TURRET_DRIVER_SRX_ID = 7;
    public static final int BOTTOM_SHOOTER_FALCON_ID = 22;
    public static final int TOP_SHOOTER_FALCON_ID = 21;

  }

}
