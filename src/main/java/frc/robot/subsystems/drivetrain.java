/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new drivetrain.
   */  
  private WPI_TalonFX leftPrimary = new WPI_TalonFX(DriveConstants.LEFT_MOTOR1_ID);
  private WPI_TalonFX leftFollow = new WPI_TalonFX(DriveConstants.LEFT_MOTOR2_ID);
  private WPI_TalonFX rightPrimary = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR1_ID);
  private WPI_TalonFX rightFollow = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_2_ID);
  
  public Drivetrain() {
    shuffleInit();
    leftPrimary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    // The motors on the left side of the drive.
    leftPrimary.setInverted(false);
    leftFollow.follow(leftPrimary);
    leftFollow.setInverted(InvertType.FollowMaster);
    // The motors on the right side of the drive.
    rightPrimary.setInverted(true);//INVERT INVERSION
    rightFollow.follow(rightPrimary);
    rightFollow.setInverted(InvertType.FollowMaster);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_WHEEL_ROTATION);
    m_rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_WHEEL_ROTATION);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    m_drive = new DifferentialDrive(leftPrimary, rightPrimary);
    m_drive.setRightSideInverted(false);

  }
  // The robot's drive
  private final DifferentialDrive m_drive;
  

  // The left-side drive encoder
  private final EncoderFalcon m_leftEncoder = new EncoderFalcon(leftPrimary);

  // The right-side drive encoder
  private final EncoderFalcon m_rightEncoder = new EncoderFalcon(rightPrimary);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   * 
   * @return
   */
  public void DriveSubsystem() {
    // Sets the distance per pulse for the encoders

    m_leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_WHEEL_ROTATION);
    m_rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_WHEEL_ROTATION);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                      m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private void shuffleInit(){
   // ShuffleBoardTab 
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
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
  public void tankDrive(double left, double right){
    m_drive.tankDrive(left , right, true);//this is doing the input squaring thing 
  }

  public void curveDrive(double xSpeed, double zRotation, Boolean isQuickTurn){
    m_drive.curvatureDrive(xSpeed, zRotation,(boolean) isQuickTurn);//needs testing
  }
  /**DONT USE
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftPrimary.setVoltage(leftVolts);
    rightPrimary.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public EncoderFalcon getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public EncoderFalcon getRightEncoder() {
    return m_rightEncoder;
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
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.IS_GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.IS_GYRO_REVERSED ? -1.0 : 1.0);
  }


}
