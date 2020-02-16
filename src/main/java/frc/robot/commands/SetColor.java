package frc.robot.commands;

//import static edu.wpi.first.wpilibj.templates.commandbased.Constants.colors;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.FalconClosedLoop;
import frc.robot.subsystems.dashTab.box;
import frc.robot.Utility.dll;
import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.ColorMatchResult;

import frc.robot.dash;
import frc.robot.Constants.ColorConst;

import com.revrobotics.ColorMatch;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class SetColor extends CommandBase {

  private ColorWheel ColorWheelHere;
  private box dashBox;

  public SetColor(dash dashboard) {
    this.ColorWheelHere = new ColorWheel();
    this.dashBox = dashboard.getDetectColorWheel();
  }

  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final String gameData = DriverStation.getInstance().getGameSpecificMessage();
  public int speedLevel = 0;
  public dll colors = new dll();

  @Override
  public void initialize() {

    colors.append("R");
    colors.append("G");
    colors.append("B");
    colors.append("Y");

    //// makes loop
    dll.Node last = colors.head;
    while (last.next != null)
      last = last.next;
    last.next = colors.head;
    colors.head.prev = last;
    //// loop

    m_colorMatcher.addColorMatch(ColorConst.Blue);
    m_colorMatcher.addColorMatch(ColorConst.Green);
    m_colorMatcher.addColorMatch(ColorConst.Red);
    m_colorMatcher.addColorMatch(ColorConst.Yellow);

  }

  public int measure() {
    Color detectedColor = ColorWheelHere.getReadColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    /**
     * switch ((String) match.color){ case Blue: colorString = "B" case Red:
     * colorString = "R" case Green: colorString = "G" case Yellow: colorString =
     * "Y" }
     */

    if (match.color == ColorConst.Blue) {
      colorString = "B";
    } else if (match.color == ColorConst.Red) {
      colorString = "R";
    } else if (match.color == ColorConst.Green) {
      colorString = "G";
    } else if (match.color == ColorConst.Yellow) {
      colorString = "Y";
    } else {
      colorString = "U";
    }

    int forwardCount = 0;
    int backwardsCount = 0;
    dll.Node CurrScroll = colors.head;
    while (CurrScroll.data != colorString) {
      CurrScroll = CurrScroll.next;
    }
    dll.Node Currforward = CurrScroll;
    dll.Node CurrBack = CurrScroll;

    while (!Currforward.data.equals(gameData)) {
      Currforward = Currforward.next;
      forwardCount++;
    }
    while (!CurrBack.data.equals(gameData)) {
      CurrBack = CurrBack.prev;
      backwardsCount--;
    }
    if (Math.abs(forwardCount) > Math.abs(backwardsCount)) {
      speedLevel = backwardsCount;
    } else if (Math.abs(forwardCount) < Math.abs(backwardsCount)) {
      speedLevel = forwardCount;
    } else if (backwardsCount == 0 & forwardCount == 0) {
      this.end(true);

    } else

    {
      speedLevel = forwardCount;
    }
    return speedLevel;
  }
private void setStuff(){
      speedLevel = measure();
    ColorWheelHere.setDistanceFrom(speedLevel);
    dashBox.set(speedLevel * 100 / 4);
}
  @Override
  public void execute() {
    setStuff();
  }

  @Override
  public void end(boolean interrupted) {
    ColorWheelHere.setDistanceFrom(1);
    
    ColorWheelHere.setDistanceFrom(0);
    while (measure() != 0){
      setStuff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}