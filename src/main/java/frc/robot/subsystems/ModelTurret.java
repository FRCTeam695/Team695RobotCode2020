/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
enum TurretServoType {
  X,Y;
}
public class ModelTurret extends SubsystemBase {
  private class ServoAndMaxPosition {
    public Servo AssociatedServo;
    public double maxPosition;
    ServoAndMaxPosition(TurretServoType servoToSet) {
      switch (servoToSet) {
        case X: {
          this.AssociatedServo = XServo;
          this.maxPosition = Constants.TURRET_HORIZONTAL_MAX_POSITION;
        }
        default: {
          this.AssociatedServo = YServo;
          this.maxPosition = Constants.TURRET_VERTICAL_MAX_POSITION;
        }
      }
    }
  }
  private Servo XServo;
  private Servo YServo;
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;

  /**
   * Creates a new ModelTurret.
   */
  public ModelTurret(NetworkTableInstance RobotMainNetworkTableInstace,int xServoChannel,int yServoChannel) {
    this.XServo = new Servo(xServoChannel);
    this.YServo = new Servo(yServoChannel);
    this.LimeLight = RobotMainNetworkTableInstace.getTable("limelight");
    this.LimeLightAzimuth = LimeLight.getEntry("tx");
		this.LimeLightCoPolar = LimeLight.getEntry("ty");
		this.LimeLightContourArea = LimeLight.getEntry("ta"); 		
  }
  private ServoAndMaxPosition getServoObjectAndUpperBoundFromServoType(TurretServoType ServoType) {
    return new ServoAndMaxPosition(ServoType);
  }
  public double getHorizontal() {
    return XServo.getAngle();
  }

  public double getVertical() {
    return YServo.getAngle();
  }

  public double getAzimuthToTarget(){
    return LimeLightAzimuth.getDouble(0.0);
  }
  public double getCoPolarToTarget(){
    return LimeLightCoPolar.getDouble(0.0);
  }
  public double getContourArea(){
    return LimeLightContourArea.getDouble(0.0);
  }
  private double getClosestBound(double position, double upperBound) {
    Double[] bounds = {0.0,0.0}; //zeroth and first index correspond to lower and upper bound respectively. 
    bounds[1] = upperBound;
    //replace contents of bounds with the distance to each bound.
    for (int i=0; i < bounds.length;++i) {
      double distanceToBound = Math.abs(bounds[i]-position);
      bounds[i] = distanceToBound;
    }
    return Collections.min(Arrays.asList(bounds));
  }


  private void setServoPosition(double position,Servo ServoObjectToModify, double ServoPositionUpperbound) throws Exception {
    if(0 <= position && position <= ServoPositionUpperbound) {
      ServoObjectToModify.setAngle(position);
    }
    else {
      ServoObjectToModify.setAngle(getClosestBound(position, ServoPositionUpperbound));
      throw(new Exception("Attempt to set turret to an excessive position "+((Double) position).toString()));
    }
  }
  private void setServoPosition(double position, ServoAndMaxPosition ServoAndMaxPositionAltered) throws Exception {
    setServoPosition(position, ServoAndMaxPositionAltered.AssociatedServo, ServoAndMaxPositionAltered.maxPosition);
  }
  private void setServoPosition(double position,TurretServoType ServoToSet) throws Exception{
    setServoPosition(position,new ServoAndMaxPosition(ServoToSet));
  }
  
  public void setHoriztonal(double position) throws Exception{
    setServoPosition(position, TurretServoType.X);
  }

  public void setVertical(double position) throws Exception{
    setServoPosition(position, TurretServoType.Y);
  }

  private void centerTurretServo(TurretServoType ServoToCenter) {
    ServoAndMaxPosition ServoObjectToModifyAndUpperBound = getServoObjectAndUpperBoundFromServoType(ServoToCenter);
    ServoObjectToModifyAndUpperBound.AssociatedServo.setAngle(ServoObjectToModifyAndUpperBound.maxPosition/2);
  }
  public void centerTurretBothAxes() {
    centerTurretServo(TurretServoType.X);
    centerTurretServo(TurretServoType.Y);
  }

  private void incrementTurretPosition(TurretServoType ServoToIncrement, double incrementAmount) throws Exception{
    ServoAndMaxPosition maxPosAndServo = getServoObjectAndUpperBoundFromServoType(ServoToIncrement);
    setServoPosition(maxPosAndServo.AssociatedServo.getAngle()+incrementAmount,maxPosAndServo);
  }
  public void incrementHorizontalPosition(double incrementAmount) throws Exception {
    incrementTurretPosition(TurretServoType.X, incrementAmount);
  }
  public void incrementVerticalPosition(double incrementAmount) throws Exception {
    incrementTurretPosition(TurretServoType.Y, incrementAmount);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
