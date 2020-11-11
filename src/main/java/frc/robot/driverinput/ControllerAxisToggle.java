/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.driverinput;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class ControllerAxisToggle extends Button {
  private Joystick ControllerWatched;
  private int axisId;
  private double lastAxisReading = 0;
  /**
   * Creates a new AxisEnableDetector.
   */
  public ControllerAxisToggle(Joystick ControllerWatched, int axisId) {
    this.ControllerWatched = ControllerWatched;
    this.axisId = axisId;
  }



  public boolean get() {
    double axisReading = ControllerWatched.getRawAxis(axisId);
    if (axisReading < 0.0001) {
      axisReading = 0;
    }
    boolean enableStateToggled = (axisReading != lastAxisReading) && ((axisReading == 0) || (lastAxisReading == 0));
    lastAxisReading = axisReading;
    return enableStateToggled;
  }
}
