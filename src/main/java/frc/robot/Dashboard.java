/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Utility.Tuple;
import frc.robot.subsystems.dashTab;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.dashTab.box;
import frc.robot.Utility.Tuple;

/**
 * Add your docs here.
 */
public class Dashboard {
	private final String defTabTitle = "default";
	private final dashTab thisisatab = new dashTab(defTabTitle);
	private box gyroBox;
	private box fixedColorWheel;
	private box detectColorWheel;
	private box focusing;
	private box searching;
	private box turretTopMotorSpeed;
	private box turretBottomMotorSpeed;

	public Dashboard(){

	}
/**
 * 
 * @param gyro instance of gyro
 */
	public void initDash() {
		//c
		thisisatab.selectTab();
		/**gyroBox = thisisatab.newBox("a",0.0, BuiltInWidgets.kTextView,new Tuple<Integer,Integer>(1,1), new Tuple<Integer,Integer>(0,0));
		thisisatab.newSensor("Gyro",gyro, BuiltInWidgets.kGyro,new Tuple<Integer,Integer>(1,1), new Tuple<Integer,Integer>(2,0));
		fixedColorWheel = thisisatab.newBox("Fixed Rotations", BuiltInWidgets.kNumberBar, new Tuple<Integer,Integer>(1,1), new Tuple<Integer,Integer>(9,3),Map.of("min",0,"Max",100));
		detectColorWheel = thisisatab.newBox("To Color", BuiltInWidgets.kNumberBar, new Tuple<Integer,Integer>(1,1), new Tuple<Integer,Integer>(8,3),Map.of("min",0,"Max",100));**/
		focusing = thisisatab.newBox("Target Focused",false,BuiltInWidgets.kBooleanBox,new Tuple<Integer,Integer>(1,1),new Tuple<Integer,Integer>(2,6));
		searching = thisisatab.newBox("Target Found",false,BuiltInWidgets.kBooleanBox,new Tuple<Integer,Integer>(1,1),new Tuple<Integer,Integer>(4,6));
		turretTopMotorSpeed = thisisatab.newBox("Turret Top Motor", 0.0, BuiltInWidgets.kNumberBar, new Tuple<Integer,Integer>(1,1), new Tuple<Integer,Integer>(9,3),Map.of("min",0,"Max",1));
		turretBottomMotorSpeed = thisisatab.newBox("Turret Bottom Motor", 0.0, BuiltInWidgets.kNumberBar, new Tuple<Integer,Integer>(1,1), new Tuple<Integer,Integer>(10,3),Map.of("min",0,"Max",1));

	}

	public box getGyroBox() {
		return gyroBox;
	}
	public box getFixedColorWheel(){
		return fixedColorWheel;
	}
	public box getDetectColorWheel(){
		return detectColorWheel;
	}
	public box getFocusing(){
		return focusing;
	}
	public box getSearching(){
		return searching;
	}
	public box getTurretTopMotorSpeed(){
		return turretTopMotorSpeed;
	}
	public box getTurretBottomMotorSpeed(){
		return turretBottomMotorSpeed;
	}

}
