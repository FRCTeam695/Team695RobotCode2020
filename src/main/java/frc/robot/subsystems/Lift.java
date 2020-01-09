/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;


public class Lift extends SubsystemBase {
  /**
   * Creates a new Lift.
   */
  private Solenoid Lift;
  public Lift(int toggleValue) {
    this.Lift = new Solenoid(toggleValue);
    setLiftPosition(false);
  }

  public boolean getLiftPostion() {
    return Lift.get();
  }

  public void toggleLiftPosition() {
    setLiftPosition(!getLiftPostion());
  }

  public void setLiftPosition(Boolean setToggleState){
    Lift.set(setToggleState);  
  }  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
