/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconClosedLoop;
import frc.robot.subsystems.ModelTurret;
import java.lang.Math;
import frc.robot.Constants;
public class AutoTurretRotation extends CommandBase {
  //private ModelTurret TurretControlled;
  private FalconClosedLoop Loop1;
  private FalconClosedLoop Loop2;

  private int panDirection = 1;
  /**
   * Creates a new SetTurretRotation.
   */
  public AutoTurretRotation(FalconClosedLoop Loop1, FalconClosedLoop Loop2) {
    this.Loop1 = Loop1;
    this.Loop2 = Loop2;
    addRequirements(Loop1, Loop2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    
  }

  //second version of execute
  @Override
  public void execute() {
    Loop2.setPosition(0);
    if()
    try{
      Loop1.setTurretVelocity(15);
    }
    catch(IllegalArgumentException PositionOverflow) {}
  }

  public boolean endCommand() {
    return Loop1.getContourArea() > 0.001;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand();
  }
}
