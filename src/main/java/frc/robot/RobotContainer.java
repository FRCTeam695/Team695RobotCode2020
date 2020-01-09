/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Motors RobotDriveMotors = new Motors();
	private Joystick ControllerDrive = new Joystick(0);
  private final TankDrive ActivateTankDrive = new TankDrive(RobotDriveMotors,ControllerDrive,1,5);
  private final JoystickButton AButton = new JoystickButton(ControllerDrive,1);

  private final JoystickButton XButton = new JoystickButton(ControllerDrive,3);
  private final MattDrive ActivateMattDrive = new MattDrive(RobotDriveMotors,ControllerDrive,1,4);
  private final JoystickButton YButton = new JoystickButton(ControllerDrive,4);
  private final ModelTurret Turret = new ModelTurret(2,3);
  private final CompressorController Compressor = new CompressorController();
  private final HatchGrabber HatchSolenoid = new HatchGrabber(0);
  private final Lift ToggleLiftPosition = new Lift(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //enable compressor
    new InstantCommand(Compressor::enableCompressor,Compressor).schedule();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    XButton.whenPressed(new SetTurretRotation(Turret,45.0,45.0));
    YButton.whenPressed(new InstantCommand(HatchSolenoid::toggleHatchState, HatchSolenoid));
    AButton.whenPressed(new InstantCommand(ToggleLiftPosition::toggleLiftPosition, ToggleLiftPosition));
  }


  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return ActivateMattDrive;
  }
}
