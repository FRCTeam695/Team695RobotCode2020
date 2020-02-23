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
import frc.robot.Constants;

public class IntakeMotor extends SubsystemBase {
  private VictorSPX ControlledMotor;
  /**
   * Creates a new IntakeMotor.
   */
  public IntakeMotor(int deviceNumber) {
    this.ControlledMotor = new VictorSPX(deviceNumber);
  }

  public void setPower(double powerPercent){
    ControlledMotor.set(ControlMode.PercentOutput, powerPercent);
  }
  public void setPowerUnrestricted(double powerPercent) {
    ControlledMotor.set(ControlMode.PercentOutput, powerPercent);
  }

  public void spinIntake(boolean boo){
    if(boo)
      setPower(-.5);
    else
      setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
