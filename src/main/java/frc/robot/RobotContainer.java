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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.commands.Driving.*;
import frc.robot.commands.Driving.ConventionalDrive.AutonomousMove;
import frc.robot.commands.Driving.ConventionalDrive.ConventionalArcadeDrive;
import frc.robot.commands.HopperDriver.HopperAutonomous;
import frc.robot.commands.HopperDriver.VictorControlJoystickAxis;
import frc.robot.commands.Trajectory.*;
import frc.robot.commands.Turret.*;
import frc.robot.driverinput.LogitechF310;
import frc.robot.Utility.Tuple;
//import frc.robot.commands.*; commands doesn't exist apparently was causing a compile failur
import frc.robot.subsystems.*;
import frc.robot.subsystems.dashTab.box;
import edu.wpi.first.wpilibj.Joystick;
import  edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants.AuxiliaryMotorIds;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  // The robot's subsystems and commands are defined here...
  //********************************************/
  //SUBSYSTEMS INITIALIZED & CONSTRUCTED BELOW:
  //*******************************************/
  //private final Drivetrain Drivetrain_inst = new Drivetrain();
  private final CompressorController Compressor_inst = new CompressorController();
  // Create a voltage constraint to ensure we don't accelerate too fast
  //private final SimpleMotorFeedforward forwardMotor = new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter);
  //private final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(forwardMotor, AutoConstants.kDriveKinematics, 10);
  private final AdjustableVictor BallHopperVictor = new AdjustableVictor(AuxiliaryMotorIds.HOPPER_VICTOR_ID);
  private final IntakeRake IntakeRake_Inst = new IntakeRake();
  private final Dashboard Dashboard_Inst = new Dashboard();
  
  // Create config for trajectory
  //private final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  //private final TrajectoryCommand trajectory1 = new TrajectoryCommand("src/main/deploy/paths/FindBall.wpilib.json", Drivetrain_inst);
  //tirrets
  private final Turret Turret_Inst = new Turret(RobotMainNetworkTableInstance, AuxiliaryMotorIds.TURRET_DRIVER_SRX_ID);
  private final ConventionalDriveTrain ConventionalDriveTrain_Inst = new ConventionalDriveTrain();

  //***************************************************************************/
  //USERINPUT STUFF (CONTROLLERS, JOYSTICK BUTTONS) INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
  private final LogitechF310 ControllerDrive = new LogitechF310(new Joystick(0));
  
  private final LogitechF310 ControllerShoot = new LogitechF310(new Joystick(1));
  //***************************************************************************/
  //COMMANDS INIT & CONSTRUCTED BELOW:
  //***************************************************************************/

  private final AutonomousMove moveAutonomous = new AutonomousMove(ConventionalDriveTrain_Inst);
  //private final DriveModeController DriveModeController_Inst = new DriveModeController(Drivetrain_inst, ControllerDrive);
  private final AutoTurretRotation AutoTurretRotation_inst = new AutoTurretRotation(Turret_Inst);
  private final TurretFocusPID TurretFocusPID_inst = new TurretFocusPID(Turret_Inst,new PIDController(0.1, 0.00025, 0));
  private final TurretFocusPIDAuton TurretFocusPIDAuton_Inst = new TurretFocusPIDAuton(Turret_Inst,BallHopperVictor,new PIDController(0.1, 0.001, 0));
  private final SequentialCommandGroup TurretGroup = new SequentialCommandGroup(AutoTurretRotation_inst);
  private final ConventionalArcadeDrive ConventionalArcadeDrive_Inst = new ConventionalArcadeDrive(ConventionalDriveTrain_Inst, ControllerDrive);
  //auton
  //private final SequentialCommandGroup sequentialTrajectory = new SequentialCommandGroup(traWjectory1.Runner());
  private final ParallelCommandGroup DaytonParallel = new ParallelCommandGroup(TurretFocusPIDAuton_Inst);
  //private final SequentialCommandGroup DaytonAutonomous = new SequentialCommandGroup(DaytonParallel);

  //teleop

  //ugly rake solution:  
  //private final RakeAdjusterUgly UglyRakeAdjuster = new RakeAdjusterUgly()
  //private final ParallelCommandGroup ContinuousTeleop = new ParallelCommandGroup(ConventionalArcadeDrive_Inst,UglyRakeAdjuster);
  //
  private final ParallelCommandGroup ContinuousTeleop = new ParallelCommandGroup();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  

    //enable compressor
    new InstantCommand(Compressor_inst::enableCompressor,Compressor_inst).schedule();
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //XButton.whenPressed(() -> DriveModeController_Inst.toggleDrive());
    ControllerDrive.RightTriggerAsButton.whenPressed(new InstantCommand(IntakeRake_Inst::enableRake,IntakeRake_Inst));
    ControllerDrive.RightTriggerAsButton.whenReleased(new InstantCommand(IntakeRake_Inst::disableRake,IntakeRake_Inst));

    ControllerDrive.RightBumper.whenPressed(new InstantCommand(IntakeRake_Inst::setDirectionCounterClockwise,IntakeRake_Inst));
    ControllerDrive.RightBumper.whenReleased(new InstantCommand(IntakeRake_Inst::setDirectionClockwise,IntakeRake_Inst));

    ContinuousTeleop.addCommands(ConventionalArcadeDrive_Inst);

    //ControllerShoot.AButton.whenPressed(() -> TurretFocusPID_inst.stopCommand());
    ControllerShoot.BButton.whenPressed(TurretGroup);
    ContinuousTeleop.addCommands(new VictorControlJoystickAxis(BallHopperVictor, ControllerShoot.LeftJoystick));
  }

  public Command getAutonomousCommand() {
    // Add kinematics to ensure max speed is actually obeyed
    //config.setKinematics(AutoConstants.kDriveKinematics);
    // Apply the voltage constraint
    //config.addConstraint(autoVoltageConstraint);
    return DaytonParallel;
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {

    Dashboard_Inst.initDash();
    return ContinuousTeleop;
  }
}
//tsest