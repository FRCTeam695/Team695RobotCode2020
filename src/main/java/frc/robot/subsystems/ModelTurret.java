 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.Interval;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;


public class ModelTurret extends SubsystemBase {
  private static final double TURRET_VERTICAL_MAX_POSITION = 100;
  private static final double TURRET_HORIZONTAL_MAX_POSITION = 200;
  private ServoWrapper XServo;
  private ServoWrapper YServo;
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;

  private class ServoWrapper extends Servo{
    double maximumPosition;
    double currentPosition = 0;
    Interval ValidPositionInterval;
    ServoWrapper(int pwmId, double maximumPosition) {
      super(pwmId);
      this.maximumPosition = maximumPosition;
      this.ValidPositionInterval = new Interval(0.0, maximumPosition);
    }
    ServoWrapper(int pwmId, double maximumPosition,double startingPosition) {
      this(pwmId, maximumPosition);
      setAngle(startingPosition);
    }

    @Override
    public void setAngle(double position){
      if (!ValidPositionInterval.contains(position)) {
        if (position < 0) {
          this.setAngle(0);
        } else if (position > maximumPosition) {
          this.setAngle(maximumPosition);
        }
        throw new IllegalArgumentException("Position " +((Double) position).toString() + " is invalid (∉" + ValidPositionInterval.toString() + ")");
      }
      super.setAngle(position);
      currentPosition = position;
    }
    @Override
    public double getAngle() {
      return currentPosition;
    }
    @Override
    public void set(double percentageSet) throws IllegalArgumentException {
      Interval zeroOneClosedInterval = new Interval(0, 1);
      if (!zeroOneClosedInterval.contains(percentageSet)) {
        if (percentageSet < 0) {
          this.set(0);
        } else if (percentageSet > 1) {
          this.set(1);
        }        
        throw new IllegalArgumentException("Position " +((Double) percentageSet).toString() + " is invalid (∉" + zeroOneClosedInterval.toString() + ")");
      }
      super.set(percentageSet);
      currentPosition = percentageSet*maximumPosition;      
    }
    @Override
    public double get() {
      return currentPosition/maximumPosition;
    }
    public void centerServo(){
      setAngle(maximumPosition/2);
    }

    public void incrementPosition(double positionIncrement) throws IllegalArgumentException {
      setAngle(currentPosition+positionIncrement);
    }
  }
  /**
   * Creates a new ModelTurret.
   */
  public ModelTurret(NetworkTableInstance RobotMainNetworkTableInstace,int xServoChannel,int yServoChannel) {
    this.XServo = new ServoWrapper(xServoChannel,TURRET_HORIZONTAL_MAX_POSITION);
    this.YServo = new ServoWrapper(yServoChannel,TURRET_VERTICAL_MAX_POSITION);
    this.LimeLight = RobotMainNetworkTableInstace.getTable("limelight");
    this.LimeLightAzimuth = LimeLight.getEntry("tx");
		this.LimeLightCoPolar = LimeLight.getEntry("ty");
		this.LimeLightContourArea = LimeLight.getEntry("ta"); 		
  }

  public double getHorizontal() {
    return XServo.getAngle();
  }

  public double getVertical() {
    return YServo.getAngle();
  }

  public double getAzimuth(){
    return LimeLightAzimuth.getDouble(0.0);
  }
  public double getCoPolar(){
    return LimeLightCoPolar.getDouble(0.0);
  }
  public double getContourArea(){
    return LimeLightContourArea.getDouble(0.0);
  }

  public void setXServoPosition(double position) throws IllegalArgumentException {
    XServo.setAngle(position);
  }

  public void setYServoPosition(double position) throws IllegalArgumentException {
    YServo.setAngle(position);
  }

  public void incrementXServoPosition(double incrementAmount) throws IllegalArgumentException {
    XServo.incrementPosition(incrementAmount);
  }

  public void incrementYServoPosition(double incrementAmount) throws IllegalArgumentException {
    YServo.incrementPosition(incrementAmount);
  }

  public void centerBothAxes() {
    XServo.centerServo();
    YServo.centerServo();
  }

  public void endOperation() {
    XServo.centerServo();
    YServo.setAngle(100);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
