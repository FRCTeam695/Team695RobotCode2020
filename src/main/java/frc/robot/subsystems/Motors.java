/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motors extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private VictorSPX motorL1 = new VictorSPX(1);
	private VictorSPX motorL2 = new VictorSPX(2);
	private VictorSPX motorR1 = new VictorSPX(3);
	private VictorSPX motorR2 = new VictorSPX(4);
  

  private double gain = 1;

	public void changeGain(double changeGainTo){
		gain = changeGainTo;
	}

  public Motors() {

  }

  public void setPower(double powerRight,double powerLeft) {
		double motorPowerMultiplier = .7*gain;
		double finalRightPower = -1 * powerRight*motorPowerMultiplier;
		double finalLeftPower = powerLeft*motorPowerMultiplier;
		motorR1.set(ControlMode.PercentOutput, finalRightPower);
    motorR2.set(ControlMode.PercentOutput, finalRightPower);
    motorL1.set(ControlMode.PercentOutput, finalLeftPower);
		motorL2.set(ControlMode.PercentOutput, finalLeftPower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
