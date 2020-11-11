package frc.robot.enums;

public enum RotationDirection {
    COUNTER_CLOCKWISE(-1), CLOCKWISE(1),NO_MODIFICATION(1);
    public final int SIGN_MODIFIER;
    private RotationDirection(int rotationDirectionModifier) {
      this.SIGN_MODIFIER = rotationDirectionModifier;
    }
}