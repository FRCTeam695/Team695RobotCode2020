/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ConventionalDriveTrain extends SubsystemBase {
  TalonFX RightFront,RightBack,LeftFront,LeftBack;
  /**
   * Creates a new ConventionalDriveTrain.
   */
  public ConventionalDriveTrain() {
    RightFront = new TalonFX(Constants.DriveConstants.RIGHT_MOTOR1_ID);
    RightBack  = new TalonFX(Constants.DriveConstants.RIGHT_MOTOR_2_ID);

    LeftFront  = new TalonFX(Constants.DriveConstants.LEFT_MOTOR1_ID);
    LeftBack   = new TalonFX(Constants.DriveConstants.LEFT_MOTOR2_ID);

  }
  public void setPowers(double right,double left) {
    RightFront.set(ControlMode.PercentOutput,right);
    RightBack.set(ControlMode.PercentOutput,right);

    LeftFront.set(ControlMode.PercentOutput,left);
    LeftBack.set(ControlMode.PercentOutput,left);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
