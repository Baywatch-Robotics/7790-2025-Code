package frc.robot.subsystems.Coral;

import java.util.HashSet;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;

public class CoralCommandMemoryCell extends SubsystemBase {
    // Current selections
   private String source = null;   // "left" or "right"
   private String side = null;     // "side1" to "side6"
   private String position = null; // "L1" to "L4"

   // Memory for used positions
   private HashSet<String> usedPositions = new HashSet<>();

   // Manual override flag
   private boolean manualOverride = false;


    Elastic.Notification selectionNotification = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "All selections must be made before locking in.", "Be Better!");

    Elastic.Notification repeatNotification = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Error: Position " + position + " on " + side + " has already been used.", "Override If Needed");


    // Lock-in button action
    public void lockInSelection() {
        if (source == null || side == null || position == null) {
            Elastic.sendNotification(selectionNotification);
            return;
        }

        String command = source + "-" + side + "-" + position;

    if (usedPositions.contains(command) && !manualOverride) {
        Elastic.sendNotification(repeatNotification);
    } 
    else {
        if (manualOverride) {
            usedPositions.add(command); // Only add to used positions if not overriding
        }
        System.out.println("Command locked in: " + command);
        resetPosition(); // Reset the position for quick adjustments
    }
}

// Manual override methods
public void enableManualOverride() {
    manualOverride = true;
    System.out.println("Manual override enabled.");
}

public void disableManualOverride() {
    manualOverride = false;
    System.out.println("Manual override disabled.");
}

// Button press handlers
public void setSource(String newSource) {
    source = newSource;
    System.out.println("Source set to: " + source);
}

public void setSide(String newSide) {
    side = newSide;
    System.out.println("Side set to: " + side);
}

public void setPosition(String newPosition) {
    position = newPosition;
    System.out.println("Position set to: " + position);
}

// Reset only the position for quick adjustments
private void resetPosition() {
    position = null;
}

// For debugging and verification
public void printUsedPositions() {
    System.out.println("Used positions: " + usedPositions);
}

// Example main method to demonstrate usage
public static void main(String[] args) {
    CoralCommandMemoryCell buttonBox = new CoralCommandMemoryCell();

    // Simulate button presses
    buttonBox.setSource("left");
    buttonBox.setSide("side3");
    buttonBox.setPosition("L2");
    buttonBox.lockInSelection(); // Lock in the command

    buttonBox.setPosition("L2"); // Reuse position
    buttonBox.lockInSelection(); // Should show an error

    // Enable manual override and lock in again
    buttonBox.enableManualOverride();
    buttonBox.lockInSelection(); // Should bypass the restriction

    buttonBox.disableManualOverride();
    buttonBox.setSide("side4");
    buttonBox.setPosition("L3");
    buttonBox.lockInSelection(); // Lock in another command

    // Print used positions for verification
    buttonBox.printUsedPositions();
}
}