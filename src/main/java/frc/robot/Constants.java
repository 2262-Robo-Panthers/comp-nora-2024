package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkBase.IdleMode;

public final class Constants {
  public static class OIConstants {
    public static final double kDeadband = 0.1;

    public static class USB {
      public static final int kDriverControllerPort = 0;
      public static final int kEndEffectorControllerPort = 1;
    }
  }

  public static class DriveConstants {
    public static final double kMaxSpeedLin_m_s = 2.4;
    public static final double kMaxSpeedAng_rad_s = 2 * Math.PI;

    public static final double kSlewRateMovement = 1.8;  // 180 %/s
    public static final double kSlewRateDirection = 1.2; // 1.2 rad/s
    public static final double kSlewRateRotation = 2.0;  // 200 %/s

    public static final boolean kIsGyroReversed = false;

    public static final double kTrackWidth  = Units.inchesToMeters(25.0); // Distance between left and right wheels
    public static final double kTrackLength = Units.inchesToMeters(24.5); // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kTrackLength / 2, kTrackWidth / 2),
      new Translation2d(kTrackLength / 2, -kTrackWidth / 2),
      new Translation2d(-kTrackLength / 2, kTrackWidth / 2),
      new Translation2d(-kTrackLength / 2, -kTrackWidth / 2)
    );

    static final double[] kSwerveModuleAngularOffsets = {
      0, -Math.PI / 2, Math.PI / 2, Math.PI
    };

    public static double kSwerveModuleAngularOffset(int backNotFront, int LeftNotRight) {
      return kSwerveModuleAngularOffsets
        [ModuleId.toIndex(backNotFront, LeftNotRight)];
    }

    public static class CAN {
      public static int kMotorPort(int backNotFront, int LeftNotRight, int driveNotTurn) {
        return 20 +
          ModuleId.toIndex(backNotFront, LeftNotRight, driveNotTurn);
      }
    }

    public static class ModuleId {
      public static final int Fr = 0; // front
      public static final int Bk = 1; // back
      public static final int Rt = 0; // right
      public static final int Lf = 1; // left
      public static final int Dv = 0; // drive
      public static final int Tn = 1; // turn

      public static int toIndex(int backNotFront, int LeftNotRight) {
        assert backNotFront == 0 || backNotFront == 1;
        assert LeftNotRight == 0 || LeftNotRight == 1;

        return
          ( backNotFront << 1
          | LeftNotRight << 0 );
      }

      public static int toIndex(int backNotFront, int LeftNotRight, int driveNotTurn) {
        assert driveNotTurn == 0 || driveNotTurn == 1;

        return
          ( toIndex(backNotFront, LeftNotRight) << 1
          | driveNotTurn << 0 );
      }
    }
  }

  public static class PivotConstants {
    public static final boolean kIsInverted = false;
    public static final double kMaxSpeed = 0.2;

    public static class CAN {
      public static final int kMotorPortA = 1;
      public static final int kMotorPortB = 2;
    }
  }

  public static class IntakeConstants {
    public static final boolean kIsInverted = true;
    public static final double kMaxSpeed = 0.6;

    public static class CAN {
      public static final int kMotorPortA = 3;
      public static final int kMotorPortB = 4;
    }
  }

  public static class LaunchConstants {
    public static final boolean kIsInverted = false;
    public static final double kMaxSpeed = 1.0;

    public static class CAN {
      public static final int kMotorPortA = 5;
      public static final int kMotorPortB = 6;
    }
  }

  // NeoMotorConstants and ModuleConstants taken from REV Robotics:
  // - https://github.com/REVrobotics/MAXSwerve-Java-Template

  public static final class NeoMotorConstants {
    public static final double kFreeSpeed_r_m = 5676.0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    static final int kDriveMotorPinionTeeth = 12;
    static final int kWheelBevelGearTeeth = 45;
    static final int kSpurGearTeeth = 22;
    static final int kBevelPinionTeeth = 15;
    public static final double kDriveMotorReductionFactor =
      (double) (kWheelBevelGearTeeth * kSpurGearTeeth)
      / (kDriveMotorPinionTeeth * kBevelPinionTeeth);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurnEncoderInverted = true;

    ///////////////////////////////////////////////////////

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kWheelDiameter_m = 0.0762;
    public static final double kWheelCircumference_m = kWheelDiameter_m * Math.PI;
    public static final double kDriveMotorFreeSpeed_r_s = NeoMotorConstants.kFreeSpeed_r_m / 60.0;
    public static final double kDriveWheelFreeSpeed_r_s = (kDriveMotorFreeSpeed_r_s * kWheelCircumference_m)
      / kDriveMotorReductionFactor;

    public static final double kDriveEncoderPositionFactor_m = (kWheelDiameter_m * Math.PI)
      / kDriveMotorReductionFactor;
    public static final double kDriveEncoderVelocityFactor_m_s = ((kWheelDiameter_m * Math.PI)
      / kDriveMotorReductionFactor) / 60.0;

    public static final double kTurnEncoderPositionFactor_rad = (2 * Math.PI); // radians
    public static final double kTurnEncoderVelocityFactor_rad_s = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurnEncoderPositionPIDMinInput_rad = 0;
    public static final double kTurnEncoderPositionPIDMaxInput_rad = kTurnEncoderPositionFactor_rad;

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 1 / kDriveWheelFreeSpeed_r_s;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurnMotorIdleMode = IdleMode.kBrake;

    public static final int kDriveMotorCurrentLimit_A = 40;
    public static final int kTurnMotorCurrentLimit_A = 30;
  }
}
