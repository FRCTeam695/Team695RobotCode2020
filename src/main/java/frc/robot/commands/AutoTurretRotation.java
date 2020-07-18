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
  private ModelTurret TurretControlled;


  private int panDirection = 1;
  /**
   * Creates a new SetTurretRotation.
   */
  public AutoTurretRotation(ModelTurret TurretControlled) {
    this.TurretControlled = TurretControlled;
    addRequirements(TurretControlled);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    TurretControlled.centerBothAxes();
  }

  //second version of execute
  @Override
  public void execute() {
    try{
      TurretControlled.incrementXServoPosition(10*panDirection);
    }
    catch(IllegalArgumentException PositionOverflow) {
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