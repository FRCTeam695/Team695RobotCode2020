/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.RotationDirection;;

public class AdjustableVictor extends SubsystemBase {
  private VictorSPX ControlledMotor;
  private RotationDirection CurrentDirection;
  /**
   * Creates a new IntakeMotor.
   */
  public AdjustableVictor(int deviceNumber) {
    this.ControlledMotor = new VictorSPX(deviceNumber);
    this.CurrentDirection = RotationDirection.CLOCKWISE;
  }

  public void setPower(double powerPercent){
    ControlledMotor.set(ControlMode.PercentOutput, CurrentDirection.SIGN_MODIFIER*powerPercent);
  }

  public void setDirection(RotationDirection Direction) {
    CurrentDirection = Direction;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
