/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AuxiliaryMotorIds;
import frc.robot.Constants.LimelightMounting;
public class Turret extends SubsystemBase {
  /**
   * Creates a new TurretMotor.
   */
  private TalonSRX motor;
  private FalconClosedLoop TopShooterMotor;
  private FalconClosedLoop BottomShooterMotor;
  private double gain = .3;
  //private double multiple = .3;
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;
  //private DigitalInput LimitSwitchLeft = new DigitalInput(0);
  //private DigitalInput LimitSwitchRight = new DigitalInput(0);

  public Turret(NetworkTableInstance RobotMainNetworkTableInstance, int driverNum) {
    this.LimeLight = RobotMainNetworkTableInstance.getTable("limelight");
    this.LimeLightAzimuth = LimeLight.getEntry("tx");
		this.LimeLightCoPolar = LimeLight.getEntry("ty");
    this.LimeLightContourArea = LimeLight.getEntry("ta");
    this.motor = new TalonSRX(driverNum);
    this.TopShooterMotor = new FalconClosedLoop(new TalonFX(AuxiliaryMotorIds.TOP_SHOOTER_FALCON_ID),0,ControlMode.Velocity); //The motor we use is yet to be determined.
    this.BottomShooterMotor = new FalconClosedLoop(new TalonFX(AuxiliaryMotorIds.BOTTOM_SHOOTER_FALCON_ID),0,ControlMode.Velocity); 
    this.TopShooterMotor.setClosedLoopMode(ControlMode.Velocity);
    this.BottomShooterMotor.setClosedLoopMode(ControlMode.Velocity);

  }
  
  public void setPower(double power) {
    power *= gain;

    motor.set(ControlMode. PercentOutput, power);
  }

  public boolean isTooFarLeft() {
    return false;//LimitSwitchLeft.get();
  }

  public boolean isTooFarRight() {
    return false;//LimitSwitchRight.get();
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

  public double getDistanceToContour() {
    return (82.0-LimelightMounting.LIMELIGHT_MOUNT_HEIGHT)/Math.tan(Math.toRadians(LimelightMounting.LIMELIGHT_MOUNT_ANGLE+getCoPolar()));
  }

  public double getDistanceToContourInFeet() {
    return 0.0877302343584*getDistanceToContour();
  }

  public double determineBottomMotorPercent(double distance) {
    return (41.73102 + (25.11446 - 41.73102)/(1 + Math.pow((distance/16.91975),13.40525)))/100;
  }

  public double determineBottomMotorPercent() {
    return determineBottomMotorPercent(getDistanceToContourInFeet());
  }
  public double determineTopMotorPercent(double distance) {
    return (103.9625-6.687958*distance+0.6078882*Math.pow(distance,2)-0.01795044*Math.pow(distance,3))/100;
  }

  public double determineTopMotorPercent() {
    return determineTopMotorPercent(getDistanceToContourInFeet());
  }

  public void setShooterWheelPowers(double topPercent,double bottomPercent) {
    TopShooterMotor.setVelocityPercent(topPercent);
    BottomShooterMotor.setVelocityPercent(bottomPercent);

  }
  public int getTopError() {
    return TopShooterMotor.getClosedLoopError();
  }
  public int getBottomError() {
    return BottomShooterMotor.getClosedLoopError();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
