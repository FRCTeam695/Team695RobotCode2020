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
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final int leftSideAxisID;
  private final int rightSideAxisID;
  private final Joystick Controller;
  private double forward = 0;
  private double right = 0;
  private Drivetrain Drivetrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public ArcadeDrive(Drivetrain drive,Joystick Controller, int leftSideAxisID,int rightSideAxisID) {
    this.leftSideAxisID = leftSideAxisID;
    this.rightSideAxisID = rightSideAxisID;
    this.Controller = Controller;
    this.Drivetrain = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forward = Controller.getRawAxis(leftSideAxisID)/Constants.rawAxisMaxValue;
    right = Controller.getRawAxis(rightSideAxisID)/Constants.rawAxisMaxValue;
    Drivetrain.arcadeDrive(forward, right);
    //rightPower = forward + right;
    //leftPower = forward - right;
    
    //RobotMotorControllers.setPower(leftPower, rightPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
