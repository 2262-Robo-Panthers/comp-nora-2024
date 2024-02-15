package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static class USB {
      public static final int kDriverControllerPort = 0;
    }
  }

  public static class IntakeConstants {
    public static final boolean kIsInverted = false;
    public static final double kMaxSpeed = 0.6;

    public static class CAN {
      public static final int kMotorPortA = 3;
      public static final int kMotorPortB = 4;
    }
  }

  public static class LaunchConstants {
    public static final boolean kIsInverted = false;
    public static final double kMaxSpeed = 0.8;

    public static class CAN {
      public static final int kMotorPortA = 5;
      public static final int kMotorPortB = 6;
    }
  }
}
