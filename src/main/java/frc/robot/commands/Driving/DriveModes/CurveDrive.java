/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Driving.DriveModes;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class CurveDrive extends CommandBase {
  private Drivetrain drivetrain;
  private Joystick Controller;
  private int leftAxis, rightAxis;
  private Boolean isQuickTurn = false;
  
  /**
   * Creates a new CurveDrive.
   */
  public CurveDrive(Drivetrain drive, Joystick Controller, int leftAxis, int rightAxis) {
    this.drivetrain = drive;
    this.Controller = Controller;
    this.leftAxis = leftAxis;
    this.rightAxis = rightAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  public void toggleQuickTurn(){
    isQuickTurn = !isQuickTurn;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.curveDrive(Controller.getRawAxis(leftAxis)/Constants.rawAxisMaxValue, Controller.getRawAxis(rightAxis)/Constants.rawAxisMaxValue,isQuickTurn);
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
