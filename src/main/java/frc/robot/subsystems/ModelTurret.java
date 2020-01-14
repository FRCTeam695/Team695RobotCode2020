/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

public class ModelTurret extends SubsystemBase {
  private Servo XServo;
  private Servo YServo;
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;
  private double horizontal = 0;
  private double vertical = 75;
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

  public double getHorizontal() {
    return horizontal;
  }

  public double getVertical() {
    return vertical;
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

  public void setXServoPosition(double position) throws Exception {
    if(0 <= position && position <= Constants.TURRET_HORIZONTAL_MAX_POSITION) {
      XServo.setAngle(position);
      horizontal = position;
    }
    else {
      throw(new Exception("Attempt to set turret to an unreasonably high azimuth."));
    }
  }

  public void setYServoPosition(double position) throws Exception {
    if(0 <= position && position <= Constants.TURRET_VERTICAL_MAX_POSITION) {
      YServo.setAngle(position);
      vertical = position;
    }
    else {
      throw(new Exception("Attempt to set turret to an unreasonably high elevation."));
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
