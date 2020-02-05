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
import frc.robot.subsystems.TurretMotor;

import java.lang.Math;
import frc.robot.Constants;

public class AutoTurretFocus extends CommandBase {
  /**
   * Creates a new AutoTurretFocus.
   */
  //private ModelTurret TurretControlled;
  private TurretMotor motor;
  /*public static final double VERTICAL_GAIN = -.1;
  public static final double VERTICAL_ADJUST =  -0.15;
  public static final double VERTICAL_GAIN_APPLICATION_THRESHOLD = 5;*/
  public static final double HORIZONTAL_GAIN = -5;
  public static final double HORIZONTAL_ADJUST = 2;
  public static final double HORIZONTAL_GAIN_APPLICATION_THRESHOLD = 10;
  public static final double HORIZONTAL_CAP = .75;
  private int i = 1;

  private double horizontalError;
  //private double verticalError;
  private double horizontalAdjustment;
  //private double verticalAdjustment;
  

  public AutoTurretFocus(TurretMotor motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = motor;
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    horizontalError = motor.getAzimuth();
    //verticalError = motor.getCoPolar();

    horizontalAdjustment = HORIZONTAL_ADJUST*horizontalError;
		if (horizontalError > HORIZONTAL_GAIN_APPLICATION_THRESHOLD)
		{
			horizontalAdjustment -= HORIZONTAL_GAIN;
		}
		else if (horizontalError < HORIZONTAL_GAIN_APPLICATION_THRESHOLD && horizontalError != 0)
		{
			horizontalAdjustment += HORIZONTAL_GAIN;
    }

    if(horizontalAdjustment >= HORIZONTAL_CAP)
      horizontalAdjustment = HORIZONTAL_CAP;

    /*verticalAdjustment = VERTICAL_ADJUST*verticalError;
		if (verticalError > VERTICAL_GAIN_APPLICATION_THRESHOLD)
		{
			verticalAdjustment -= VERTICAL_GAIN;
		}
		else if (verticalError < VERTICAL_GAIN_APPLICATION_THRESHOLD && verticalError != 0)
		{
			verticalAdjustment += VERTICAL_GAIN;
    }*/
    try {motor.setPower(horizontalAdjustment);}
    catch(IllegalArgumentException percentageOverFlException) {}
    //try {motor.setPower(verticalAdjustment);}
    //catch(IllegalArgumentException PositionOverflow) {}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  public void change() {
    if(i !=0 )
      i-=1;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if(i == 0) {
      i++;
      motor.endOperation();
      return true;
    }*/
    return false;
  }
}
