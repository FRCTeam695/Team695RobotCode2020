/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class dashboard extends SubsystemBase {
  /**
   * Creates a new dashboard.
   */

  public dashboard() {

  }

  public class tab {
    ShuffleboardTab thisTab;

    public tab(String title) {
      this.thisTab = Shuffleboard.getTab(title);
    }

    public class box {
      private NetworkTableEntry box;

      public box(String name, BuiltInWidgets widget, int[] size, int[] pos) {
        this.box = thisTab.add(name, 0.0).withWidget(widget).withSize(size[0],size[1]).withPosition(pos[0], pos[1]).getEntry();
      }

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
