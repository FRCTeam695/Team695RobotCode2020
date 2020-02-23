/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Driving.DriveModes;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TankDrive extends CommandBase {
  private final Drivetrain Drivetrain;
  private final Joystick Controller;
  private final int leftSideAxisID, rightSideAxisID;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(Drivetrain Drivetrain, Joystick Controller, int leftSideAxisID,int rightSideAxisID) {
    this.leftSideAxisID = leftSideAxisID;
    this.rightSideAxisID = rightSideAxisID;
    this.Controller = Controller;
    this.Drivetrain = Drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drivetrain.tankDrive(Controller.getRawAxis(leftSideAxisID)/Constants.rawAxisMaxValue, Controller.getRawAxis(rightSideAxisID)/Constants.rawAxisMaxValue);
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
