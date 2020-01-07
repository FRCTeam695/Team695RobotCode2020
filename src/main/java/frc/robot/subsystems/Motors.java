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

  public Motors() {

  }

  public void setPower(double powerRight,double powerLeft) {
    System.out.println(Double.toString(powerRight) + ", LEFT:" +Double.toString(powerLeft));
		motorR1.set(ControlMode.PercentOutput, -1 * .7*powerRight);
    motorR2.set(ControlMode.PercentOutput, -1 * .7*powerRight);
    motorL1.set(ControlMode.PercentOutput, .7*powerLeft);
		motorL2.set(ControlMode.PercentOutput, .7*powerLeft);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
