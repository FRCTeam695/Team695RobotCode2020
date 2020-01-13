/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ModelTurret;
import java.lang.Math;
import frc.robot.Constants;
public class AutoTurretRotation extends CommandBase {
  ModelTurret TurretControlled;
  Joystick Controller;
  int ButtonId;

  private double panDirection = 1;
  /**
   * Creates a new SetTurretRotation.
   */
  public AutoTurretRotation(ModelTurret TurretControlled,Joystick Controller, int ButtonId) {
    this.TurretControlled = TurretControlled;
    this.ButtonId = ButtonId;
    this.Controller = Controller;
    addRequirements(TurretControlled);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    TurretControlled.centerTurretBothAxes();
  }

  //second version of execute
  @Override
  public void execute() {
    try{
      TurretControlled.incrementHorizontalPosition(10*panDirection);
    }
    catch(Exception PositionOverflow) {
      panDirection = -panDirection;
    }
  }

  public boolean endCommand() {
    return TurretControlled.getContourArea() > 0.001;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand();
  }
}
