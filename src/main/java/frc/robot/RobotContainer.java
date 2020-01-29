/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
 */
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  // The robot's subsystems and commands are defined here...
  //***************************************************************************/
  //SUBSYSTEMS INITIALIZED & CONSTRUCTED BELOW:
  //***************************************************************************/
  private final Motors RobotDriveMotors = new Motors();
  //private final CompressorController Compressor = new CompressorController();
  //private final HatchGrabber HatchSolenoid = new HatchGrabber(0);
  //private final ModelTurret Turret = new ModelTurret(RobotMainNetworkTableInstance,2,3);
  private final BallDetector Detector = new BallDetector(0);
  private final TurretMotor Turret = new TurretMotor(RobotMainNetworkTableInstance, 5);

  //***************************************************************************/
  //USERINPUT STUFF (CONTROLLERS, JOYSTICK BUTTONS) INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
	private Joystick ControllerDrive = new Joystick(0);
  private final JoystickButton AButton = new JoystickButton(ControllerDrive,1);
  private final JoystickButton XButton = new JoystickButton(ControllerDrive,3);
  private final JoystickButton YButton = new JoystickButton(ControllerDrive,4);
  //***************************************************************************/
  //COMMANDS INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
  //private final TankDrive ActivateTankDrive = new TankDrive(RobotDriveMotors,ControllerDrive,1,5);
  // final MattDrive ActivateMattDrive = new MattDrive(RobotDriveMotors,ControllerDrive,1,4);
  private final SetColor ColorSensorUsed = new SetColor();
  private final AutoTurretRotation Finding = new AutoTurretRotation(Turret);
  private final AutoTurretFocus Focusing = new AutoTurretFocus(Turret);
  private final SequentialCommandGroup TurretGroup = new SequentialCommandGroup();
  private final FindBall ReturnBall = new FindBall(Detector/*, RobotDriveMotors*/);


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
    TurretGroup.addCommands(Finding,Focusing);
    AButton.whenPressed(TurretGroup);
    XButton.whenPressed(new InstantCommand(Focusing::change));
    //YButton.whenPressed(new InstantCommand(HatchSolenoid::toggleHatchState, HatchSolenoid));

  }



  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    ParallelCommandGroup ContinuousTeleop = new ParallelCommandGroup();
    //ContinuousTeleop.addCommands(ReturnBall);
    return ContinuousTeleop;
  }
}
//tsest