/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import  edu.wpi.first.wpilibj.controller.PIDController;

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
  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  // The robot's subsystems and commands are defined here...
  //***************************************************************************/
  //SUBSYSTEMS INITIALIZED & CONSTRUCTED BELOW:
  //***************************************************************************/
  private final Drivetrain Drivetrain_inst = new Drivetrain();
  private final CompressorController Compressor_inst = new CompressorController();
  // Create a voltage constraint to ensure we don't accelerate too fast
  private final SimpleMotorFeedforward forwardMotor = new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter);
  private final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(forwardMotor, AutoConstants.kDriveKinematics, 10);

  // Create config for trajectory
  private final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  // Trajectories
  private final TrajectoryCommand trajectory1 = new TrajectoryCommand("src/main/deploy/paths/FindBall.wpilib.json", Drivetrain_inst);
  
  
  //tirrets
  private final Turret Turret_Inst = new Turret(RobotMainNetworkTableInstance, 0);
  //***************************************************************************/
  //USERINPUT STUFF (CONTROLLERS, JOYSTICK BUTTONS) INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
  private final Joystick ControllerDrive = new Joystick(0);
  private final JoystickButton AButton = new JoystickButton(ControllerDrive,1);
  private final JoystickButton BButton = new JoystickButton(ControllerDrive,2);
  private final JoystickButton XButton = new JoystickButton(ControllerDrive,3);
  private final JoystickButton YButton = new JoystickButton(ControllerDrive,4);
  private final POVButton POVTopRight = new POVButton(ControllerDrive, 45);
  private final POVButton POVBottomLeft = new POVButton(ControllerDrive, 135);
  private final POVButton POVBottomRight = new POVButton(ControllerDrive, 225);
  private final POVButton POVTopLeft = new POVButton(ControllerDrive, 315);
  
  //***************************************************************************/
  //COMMANDS INIT & CONSTRUCTED BELOW:
  //***************************************************************************/


  private final DriveModeController DriveModeController_Inst = new DriveModeController(Drivetrain_inst, ControllerDrive);
  private final AutoTurretRotation AutoTurretRotation_inst = new AutoTurretRotation(Turret_Inst);
  private final TurretFocusPID TurretFocusPID_inst = new TurretFocusPID(Turret_Inst,new PIDController(0.1, 0.001, 0));
  private final SequentialCommandGroup TurretGroup = new SequentialCommandGroup(AutoTurretRotation_inst,TurretFocusPID_inst);

  //autonomous
  private final SequentialCommandGroup sequentialTrajectory = new SequentialCommandGroup(trajectory1.Runner());
  private final SequentialCommandGroup Dayton = new SequentialCommandGroup();

  //teleop
  private final ParallelCommandGroup ContinuousTeleop = new ParallelCommandGroup(DriveModeController_Inst);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    AButton.whenPressed(() -> TurretFocusPID_inst.stopCommand());
    Dayton.addCommands(TurretFocusPID_inst);
    Dayton.addCommands();

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
    BButton.whenPressed(TurretGroup);
    XButton.whenPressed(() -> DriveModeController_Inst.toggleDrive());


  }

  public Command getAutonomousCommand() {

    
    // Add kinematics to ensure max speed is actually obeyed
    //config.setKinematics(AutoConstants.kDriveKinematics);
    // Apply the voltage constraint
    //config.addConstraint(autoVoltageConstraint);
    return sequentialTrajectory;
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {

    return ContinuousTeleop;
  }
}
//tsest