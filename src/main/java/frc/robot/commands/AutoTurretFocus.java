/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ModelTurret;
import java.lang.Math;
import frc.robot.Constants;

public class AutoTurretFocus extends CommandBase {
  /**
   * Creates a new AutoTurretFocus.
   */
  ModelTurret TurretControlled;
  Joystick Controller;
  int ButtonId;
  private double horizontal;
  private double vertical;
  private double horizontalError;
  private double verticalError;
  private double horizontalAdjustment;
  private double verticalAdjustment;
  

  public AutoTurretFocus(ModelTurret TurretControlled,Joystick Controller, int ButtonId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.TurretControlled = TurretControlled;
    this.Controller = Controller;
    this.ButtonId = ButtonId;
    addRequirements(TurretControlled);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.horizontal = TurretControlled.getHorizontal();
    this.vertical = TurretControlled.getVertical();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    horizontalError = TurretControlled.getAzimuth();
    verticalError = TurretControlled.getCoPolar();

    horizontalAdjustment = Constants.CONSTANTY*horizontalError;
		if (horizontalError > 3.0)
		{
			horizontalAdjustment = Constants.CONSTANTY*horizontalError - Constants.GAIN;
		}
		else if (horizontalError < 3.0)
		{
			horizontalAdjustment = Constants.CONSTANTY*horizontalError + Constants.GAIN;
    }

    verticalAdjustment = Constants.CONSTANTY*verticalError;
		if (verticalError > 3.0)
		{
			verticalAdjustment = Constants.CONSTANTY*verticalError - Constants.GAIN;
		}
		else if (verticalError < 3.0)
		{
			verticalAdjustment = Constants.CONSTANTY*verticalError + Constants.GAIN;
    }
    
    horizontal += horizontalAdjustment;
    vertical += horizontalAdjustment;
    TurretControlled.setXServoPosition(horizontal);
    TurretControlled.setYServoPosition(vertical);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
