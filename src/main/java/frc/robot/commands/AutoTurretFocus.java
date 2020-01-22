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
import frc.robot.subsystems.FalconClosedLoop;
import frc.robot.subsystems.ModelTurret;
import java.lang.Math;
import frc.robot.Constants;

public class AutoTurretFocus extends CommandBase {
  /**
   * Creates a new AutoTurretFocus.
   */
  //private ModelTurret Loop;
  private FalconClosedLoop Loop1;
  private FalconClosedLoop Loop2;
  public static final double VERTICAL_GAIN = -.1;
  public static final double VERTICAL_ADJUST =  -0.15;
  public static final double VERTICAL_GAIN_APPLICATION_THRESHOLD = 5;
  public static final double HORIZONTAL_GAIN = -.2;
  public static final double HORIZONTAL_ADJUST = 0.3;
  public static final double HORIZONTAL_GAIN_APPLICATION_THRESHOLD = 10;
  private int i = 1;

  private double horizontalError;
  private double verticalError;
  private double horizontalAdjustment;
  private double verticalAdjustment;
  

  public AutoTurretFocus(FalconClosedLoop Loop1, FalconClosedLoop Loop2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Loop1 = Loop1;
    this.Loop2 = Loop2;
    addRequirements(Loop1, Loop2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    horizontalError = Loop1.getAzimuth();
    verticalError = Loop1.getCoPolar();

    //horizontalAdjustment = HORIZONTAL_ADJUST*horizontalError;
		if (horizontalError > HORIZONTAL_GAIN_APPLICATION_THRESHOLD)
		{
			horizontalError -= HORIZONTAL_GAIN;
		}
		else if (horizontalError < HORIZONTAL_GAIN_APPLICATION_THRESHOLD && horizontalError != 0)
		{
			horizontalError += HORIZONTAL_GAIN;
    }

    //verticalAdjustment = VERTICAL_ADJUST*verticalError;
		if (verticalError > VERTICAL_GAIN_APPLICATION_THRESHOLD)
		{
			verticalError -= VERTICAL_GAIN;
		}
		else if (verticalError < VERTICAL_GAIN_APPLICATION_THRESHOLD && verticalError != 0)
		{
			verticalError += VERTICAL_GAIN;
    }
    try {Loop1.setVelocity(horizontalAdjustment);}
    catch(IllegalArgumentException PositionOverflow) {}
    try {Loop2.setVelocity(verticalAdjustment);}
    catch(IllegalArgumentException PositionOverflow) {}
    
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
    return horizontalError < 10 && horizontalError > -10 && verticalError > 10 && verticalError > -10;
  }
}
