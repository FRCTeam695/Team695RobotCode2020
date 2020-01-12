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
    try{TurretControlled.setXServoPosition(Constants.TURRET_HORIZONTAL_MAX_POSITION/2);}
    catch(Exception thrown) {}
    try{TurretControlled.setYServoPosition(Constants.TURRET_VERTICAL_MAX_POSITION/2);}
    catch(Exception thrown) {}
  }

  //second version of execute
  @Override
  public void execute() {
    
    double horizontal = TurretControlled.getHorizontal();
    double vertical = TurretControlled.getVertical();


    double alteredHorizontal = horizontal+10*panDirection;
    if(0 <= alteredHorizontal && alteredHorizontal <= Constants.TURRET_HORIZONTAL_MAX_POSITION)
      horizontal = alteredHorizontal;
    else 
      panDirection = -panDirection;
    System.out.println(horizontal);
    try{TurretControlled.setYServoPosition(vertical);}
    catch(Exception exceptionThrown) {}
    try{TurretControlled.setXServoPosition(horizontal);}
    catch(Exception exceptionThrown) {}
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
