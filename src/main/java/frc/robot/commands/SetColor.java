package frc.robot.commands;

//import static edu.wpi.first.wpilibj.templates.commandbased.Constants.colors;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.ColorWheel;
import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.ColorMatchResult;

import com.revrobotics.ColorMatch;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class SetColor extends CommandBase {

  public class DLL {
    Node head; // head of list

    /* Doubly Linked list Node */
    class Node {
      String data;
      Node prev;
      Node next;

      // Constructor to create a new node
      // next and prev is by default initialized as null
      Node(String d) {
        data = d;
      }
    }

    // Adding a node at the front of the list
    public void push(String new_data) {
      /*
       * 1. allocate node 2. put in the data
       */
      Node new_Node = new Node(new_data);

      /* 3. Make next of new node as head and previous as NULL */
      new_Node.next = head;
      new_Node.prev = null;

      /* 4. change prev of head node to new node */
      if (head != null)
        head.prev = new_Node;

      /* 5. move the head to point to the new node */
      head = new_Node;
    }

    /* Given a node as prev_node, insert a new node after the given node */
    public void InsertAfter(Node prev_Node, String new_data) {

      /* 1. check if the given prev_node is NULL */
      if (prev_Node == null) {
        System.out.println("The given previous node cannot be NULL ");
        return;
      }

      /*
       * 2. allocate node 3. put in the data
       */
      Node new_node = new Node(new_data);

      /* 4. Make next of new node as next of prev_node */
      new_node.next = prev_Node.next;

      /* 5. Make the next of prev_node as new_node */
      prev_Node.next = new_node;

      /* 6. Make prev_node as previous of new_node */
      new_node.prev = prev_Node;

      /* 7. Change previous of new_node's next node */
      if (new_node.next != null)
        new_node.next.prev = new_node;
    }

    // Add a node at the end of the list
    void append(String new_data) {
      /*
       * 1. allocate node 2. put in the data
       */
      Node new_node = new Node(new_data);

      Node last = head; /* used in step 5 */

      /*
       * 3. This new node is going to be the last node, so make next of it as NULL
       */
      new_node.next = null;

      /*
       * 4. If the Linked List is empty, then make the new node as head
       */
      if (head == null) {
        new_node.prev = null;
        head = new_node;
        return;
      }

      /* 5. Else traverse till the last node */
      while (last.next != null)
        last = last.next;

      /* 6. Change the next of last node */
      last.next = new_node;

      /* 7. Make last node as previous of new node */
      new_node.prev = last;

    }

    // This function prints contents of linked list starting from the given node
    public void printlist(Node node) {
      Node last = null;
      System.out.println("Traversal in forward Direction");
      while (node != null) {
        System.out.print(node.data + " ");
        last = node;
        node = node.next;
      }
      System.out.println();
      System.out.println("Traversal in reverse direction");
      while (last != null) {
        System.out.print(last.data + " ");
        last = last.prev;
      }
    }

  }

  private ColorWheel ColorWheelHere = new ColorWheel();
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color Blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color Green = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color Red = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color Yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final String gameData = DriverStation.getInstance().getGameSpecificMessage();
  public int speedLevel = 0;
  public DLL colors = new DLL();

  @Override
  public void initialize() {

    colors.append("R");
    colors.append("G");
    colors.append("B");
    colors.append("Y");

    //// makes loop
    DLL.Node last = colors.head;
    while (last.next != null)
      last = last.next;
    last.next = colors.head;
    colors.head.prev = last;
    //// loop

    m_colorMatcher.addColorMatch(Blue);
    m_colorMatcher.addColorMatch(Green);
    m_colorMatcher.addColorMatch(Red);
    m_colorMatcher.addColorMatch(Yellow);

  }

  @Override
  public void execute() {

    Color detectedColor = ColorWheelHere.getReadColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    /**
     * switch ((String) match.color){ case Blue: colorString = "B" case Red:
     * colorString = "R" case Green: colorString = "G" case Yellow: colorString =
     * "Y" }
     */

    if (match.color == Blue) {
      colorString = "B";
    } else if (match.color == Red) {
      colorString = "R";
    } else if (match.color == Green) {
      colorString = "G";
    } else if (match.color == Yellow) {
      colorString = "Y";
    } else {
      colorString = "U";
    }

    int forwardCount = 0;
    int backwardsCount = 0;
    DLL.Node CurrScroll = colors.head;
    while (CurrScroll.data != colorString) {
      CurrScroll = CurrScroll.next;
    }
    DLL.Node Currforward = CurrScroll;
    DLL.Node CurrBack = CurrScroll;

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
    ColorWheelHere.ColorMotorSet(speedLevel);

  }

  @Override
  public void end(boolean interrupted) {
    ColorWheelHere.ColorMotorSet(1);
    // delay 2 rotations or 0.1 seconds
    ColorWheelHere.ColorMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}