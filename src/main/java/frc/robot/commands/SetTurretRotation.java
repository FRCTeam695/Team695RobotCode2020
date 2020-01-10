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

public class SetTurretRotation extends CommandBase {
  ModelTurret TurretControlled;
  Joystick Controller;
  int verticalAxisId;
  int horizontalAxisId;
  private double left = 0;
  private double up = 0;

  /**
   * Creates a new SetTurretRotation.
   */
  public SetTurretRotation(ModelTurret TurretControlled,Joystick Controller, int horizontalAxisId,int verticalAxisId) {
    this.TurretControlled = TurretControlled;
    this.horizontalAxisId = horizontalAxisId;
    this.verticalAxisId = verticalAxisId;
    this.Controller = Controller;
    addRequirements(TurretControlled);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {

    left = ((Controller.getRawAxis(horizontalAxisId)+1)*90)*1.5;
    up = (Controller.getRawAxis(verticalAxisId)+1)*90;

    TurretControlled.setXServoAngle(left);
    TurretControlled.setYServoAngle(up);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
