/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Motors;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Motors RobotMotorControllers;
  private final GenericHID.Hand LeftMotorHand;
  private final GenericHID.Hand RightMotorHand;
  private final Joystick Controller;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(Motors RobotMotorControllers,Joystick Controller, GenericHID.Hand LeftMotorHand,GenericHID.Hand RightMotorHand) {
    this.RobotMotorControllers = RobotMotorControllers;
    this.LeftMotorHand = LeftMotorHand;
    this.RightMotorHand = RightMotorHand;
    this.Controller = Controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotMotorControllers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotMotorControllers.setPower(Controller.getY(LeftMotorHand), Controller.getY(RightMotorHand));
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
