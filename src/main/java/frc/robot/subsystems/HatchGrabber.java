/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
public class HatchGrabber extends SubsystemBase {
  private Solenoid Hatch;

  /**
   * Creates a new HatchGrabber.t
   */
  public HatchGrabber(int hatchID) {
    this.Hatch = new Solenoid(hatchID);
    setHatch(false);
  }
  
  public void setHatch(Boolean setState){
    Hatch.set(setState);  
  }

  public boolean getHatchState() {
    return Hatch.get();
  }

  public void toggleHatchState() {
    setHatch(!getHatchState());
  }
}
