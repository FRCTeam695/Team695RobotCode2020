/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretMotor extends SubsystemBase {
  /**
   * Creates a new TurretMotor.
   */
  private VictorSPX motor;
  private double gain = 1;
  private double multiple = .75;
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;
  
  public TurretMotor(NetworkTableInstance RobotMainNetworkTableInstance, int motorNum) {
    this.LimeLight = RobotMainNetworkTableInstance.getTable("limelight");
    this.LimeLightAzimuth = LimeLight.getEntry("tx");
		this.LimeLightCoPolar = LimeLight.getEntry("ty");
    this.LimeLightContourArea = LimeLight.getEntry("ta");
    this.motor = new VictorSPX(motorNum);
  }
  
  public void setPower(double power) {
    power *= multiple*gain;
    motor.set(ControlMode.PercentOutput, power);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
