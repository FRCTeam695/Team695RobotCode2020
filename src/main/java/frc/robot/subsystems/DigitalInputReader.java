/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DigitalInputReader extends SubsystemBase {
  /**
   * Creates a new DigitalInputReader.
   */
  private final DigitalInput ReadDigitalInput;
  public DigitalInputReader(int digitalInputId) {
    this.ReadDigitalInput = new DigitalInput(digitalInputId);
  }


  @Override
  public void periodic() {
    System.out.println(ReadDigitalInput.get());
    // This method will be called once per scheduler run
  }
}
