/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
/**
 * COntroller button indicies:
 * A: 1
 * B: 2
 * X: 3
 * Y: 4
 * 
 * Left X-axis: 0
 * Left Y-axis: 1
 * 
 * POV:
 *      0
 *      ^
 *      |
 * 270<- -> 90
 *      |
 *      v
 *     180
 * /
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //***************************************************************************/
  //SUBSYSTEMS INITIALIZED & CONSTRUCTED BELOW:
  //***************************************************************************/
  //private final Motors RobotDriveMotors = new Motors();
  //private final CompressorController Compressor = new CompressorController();
  //private final HatchGrabber HatchSolenoid = new HatchGrabber(0);
  //private final ModelTurret Turret = new ModelTurret(2,3);

  //***************************************************************************/
  //USERINPUT STUFF (CONTROLLERS, JOYSTICK BUTTONS) INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
	private Joystick ControllerDrive = new Joystick(0);
  private final JoystickButton AButton = new JoystickButton(ControllerDrive,1);
  ////private final JoystickButton XButton = new JoystickButton(ControllerDrive,3);
  private final JoystickButton YButton = new JoystickButton(ControllerDrive,4);
  private final POVButton POVTopRight = new POVButton(ControllerDrive, 45);
  private final POVButton POVBottomLeft = new POVButton(ControllerDrive, 135);
  private final POVButton POVBottomRight = new POVButton(ControllerDrive, 225);
  private final POVButton POVTopLeft = new POVButton(ControllerDrive, 315);

  //***************************************************************************/
  //COMMANDS INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
  //private final TankDrive ActivateTankDrive = new TankDrive(RobotDriveMotors,ControllerDrive,1,5);
  // final MattDrive ActivateMattDrive = new MattDrive(RobotDriveMotors,ControllerDrive,1,4);
  //private final SetColor ColorSensorUsed = new SetColor();
  //private final SetTurretRotation ActivateTurret = new SetTurretRotation(Turret, ControllerDrive, 0, 1);
  private final FalconClosedLoop ClosedLoopTop = new FalconClosedLoop(12,0,30,ControlMode.Velocity); //The motor we use is yet to be determined.
  private final FalconClosedLoop ClosedLoopBottom = new FalconClosedLoop(10,0,30,ControlMode.Velocity); 
  //  private final EnableFalconVelocityClosedLoop ActivateClosedLoop = new EnableFalconVelocityClosedLoop(ClosedLoop,3000);
  //private final PrintControllerPOV PrintController = new PrintControllerPOV(ControllerDrive);
  private final MultipleFalconClosedVelocity shooterMultiVelocity = new MultipleFalconClosedVelocity(new double[]{0.0,0.0},new FalconClosedLoop[]{ClosedLoopTop,ClosedLoopBottom});
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //enable compressor
    //new InstantCommand(Compressor::enableCompressor,Compressor).schedule();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    POVTopRight.whenPressed(new InstantCommand(() -> {shooterMultiVelocity.incrementSpecificVelocity(0,500);}));
    POVTopLeft.whenPressed(new InstantCommand(() -> {shooterMultiVelocity.incrementSpecificVelocity(0,-500);}));
    POVBottomRight.whenPressed(new InstantCommand(() -> {shooterMultiVelocity.incrementSpecificVelocity(1,500);}));
    POVBottomLeft.whenPressed(new InstantCommand(() -> {shooterMultiVelocity.incrementSpecificVelocity(1,-500);}));

   // YButton.whenPressed(new InstantCommand(() -> {ActivateClosedLoop.incrementPosition(1);}));
    //AButton.whenPressed(new InstantCommand(() -> {ActivateClosedLoop.incrementPosition(-1);}));
  }


  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    ParallelCommandGroup ContinuousTeleop = new ParallelCommandGroup();
    //test.set(ControlMode.PercentOutput,0.5);
    ContinuousTeleop.addCommands(shooterMultiVelocity);
    return ContinuousTeleop;
  }
}
//tsest