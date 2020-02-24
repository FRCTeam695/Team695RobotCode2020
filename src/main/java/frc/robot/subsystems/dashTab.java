/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.Tuple;

public class dashTab extends SubsystemBase {
  private ShuffleboardTab thisTab;
  private String title;

  public dashTab(String title) {
    this.thisTab = Shuffleboard.getTab(title);
    this.title = title;
  }
  public void selectTab(){
    Shuffleboard.selectTab(title);
  }

  public box getBox(String name) {
    return new box(thisTab.add(name, 0).getEntry());
  }
/**
 * 
 * @param name string name
 * @param defaultValue name
 * @param widget BuiltInWidgets.
 * @param size tuple of ints
 * @param pos tuple of ints
 * @param properties Map.of("min", 0, "max", 1)
 * @return box type
 */
  public box newBox(String name,Object defaultValue, BuiltInWidgets widget, Tuple<Integer,Integer> size, Tuple<Integer,Integer> pos, Map<String,Object> properties) {
    return new box(
        thisTab.add(name, defaultValue).withWidget(widget).withProperties(properties).withSize( size.x,size.y).withPosition(pos.x, pos.y).getEntry());
  }
  public box newBox(String name, BuiltInWidgets widget, Tuple<Integer,Integer> size, Tuple<Integer,Integer> pos){
    return newBox(name, 0.0, widget, size, pos);
  }
  public box newBox(String name,Object defaultValue, BuiltInWidgets widget, Tuple<Integer,Integer> size, Tuple<Integer,Integer> pos){
    return newBox(name, defaultValue, widget, size, pos,Map.of());
  }
  public box newBox(String name, BuiltInWidgets widget, Tuple<Integer,Integer> size, Tuple<Integer,Integer> pos,Map<String,Object> properties){
    return newBox(name, 0.0, widget, size, pos,properties);
  }
  /**
   * 
   * @param name
   * @param a typecast to sendable
   * @param widget
   * @param size
   * @param pos
   */
  public void newSensor(String name, Object obj, BuiltInWidgets widget, Tuple<Integer,Integer> size, Tuple<Integer,Integer> pos) {
    thisTab.add(name, (Sendable) obj).withSize( size.x,size.y).withPosition(pos.x, pos.y);
    System.out.println("aonhonaoihntosnt");
  }

  public class box {

    public NetworkTableEntry box;

    public box(NetworkTableEntry box) {
      this.box = box;
    }

    public void set(Object hello) {
      box.forceSetValue(hello);
    }
    public void set(Double hello){
      box.forceSetDouble((double) hello);
    }
    public void set(double hello){
      box.forceSetDouble(hello);
    }
    public void set(int hello){
      box.forceSetDouble((double) hello);
    }
    public void set(Integer hello){
      box.forceSetDouble((double) hello);
    }
    public void set(String hello){
      box.forceSetString(hello);
    }
    public void set(boolean hello){
      box.forceSetBoolean(hello);
    }
    public void set(Boolean hello){
      box.forceSetBoolean((boolean) hello);
    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
