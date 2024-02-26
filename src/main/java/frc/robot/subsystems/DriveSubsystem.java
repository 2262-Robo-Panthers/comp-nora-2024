// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import static edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kZ;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.MAXSwerve.MAXSwerveModule;
import frc.robot.lib.MAXSwerve.SwerveUtils;
import frc.robot.util.FormatUtil;

public class DriveSubsystem extends SubsystemBase {
  private final ShuffleboardTab m_dashboard;

  private final double m_maxSpeedLin;
  private final double m_maxSpeedAng;
  private final boolean m_isGyroReversed;

  private final SlewRateLimiter m_limiterMovement;
  private final SlewRateLimiter m_limiterRotation;
  private final double m_slewRateDirection;

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final SwerveDriveOdometry m_odometry;
  private final SwerveDriveKinematics m_driveKinematics;

  private final MAXSwerveModule[] m_modules;

  private boolean m_isFieldRelative = false;
  private boolean m_isXFormation = false;

  private double m_currentMovementMag = 0.0;
  private double m_currentMovementDir = 0.0;

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private static final SwerveModuleState[] kXFormationStates = {
    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
    new SwerveModuleState(0.0, Rotation2d.fromDegrees(+45.0)),
    new SwerveModuleState(0.0, Rotation2d.fromDegrees(+45.0)),
    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0))
  };

  public DriveSubsystem(
    ShuffleboardTab shuffleboardTab,
    double maxSpeedLin_m_s, double maxSpeedAng_rad_s, boolean isGyroReversed,
    double slewRateMovement, double slewRateDirection, double slewRateRotation,
    SwerveDriveKinematics driveKinematics,
    MAXSwerveModule... modules
  ) {
    m_dashboard = shuffleboardTab;

    m_maxSpeedLin = maxSpeedLin_m_s;
    m_maxSpeedAng = maxSpeedAng_rad_s;
    m_isGyroReversed = isGyroReversed;

    m_limiterMovement = new SlewRateLimiter(slewRateMovement);
    m_limiterRotation = new SlewRateLimiter(slewRateRotation);
    m_slewRateDirection = slewRateDirection;

    m_modules = modules;
    m_odometry = new SwerveDriveOdometry(driveKinematics, getRotation(), getModulePositions());
    m_driveKinematics = driveKinematics;

    populateDashboard();
  }

  @Override
  public void periodic() {
    m_odometry.update(getRotation(), getModulePositions());
  }

  private void populateDashboard() {
    m_dashboard.addBoolean("IsXFormation",
      () -> m_isXFormation);

    m_dashboard.add("IsFieldRelative", m_isFieldRelative)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry()
      .andThen(x -> m_isFieldRelative = x.getBoolean());

    m_dashboard.addStringArray("Pose",
      FormatUtil.formatted(this::getPose, pose -> List.of(
        new Pair<>("x", pose.getX()),
        new Pair<>("y", pose.getY()),
        new Pair<>("θ", pose.getRotation().getDegrees())
      )));

    m_dashboard.addStringArray("Gyro",
      FormatUtil.formatted(List.of(
        new Pair<>("θ", this::getRotation_deg),
        new Pair<>("ω", this::getRotationRate_deg_s)
      )));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void usePose(Pose2d pose) {
    m_odometry.resetPosition(getRotation(), getModulePositions(), pose);
  }

  public double getRotation_deg() {
    return getRotation().getDegrees();
  }

  public double getRotationRate_deg_s() {
    return m_gyro.getRate(kZ) *  (m_isGyroReversed ? -1.0 : 1.0);
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(kZ));
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(m_modules)
      .map(MAXSwerveModule::getPosition)
      .toArray(n -> new SwerveModulePosition[n]);
  }

  public void resetEncoders() {
    for (MAXSwerveModule module : m_modules) {
      module.resetEncoders();
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    assert desiredStates.length >= m_modules.length;

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeedLin);

    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(desiredStates[i]);
    }
  }

  public void setReferencePlane(boolean isFieldRelative) {
    m_isFieldRelative = isFieldRelative;
  }

  public void setXFormation(boolean isXFormation) {
    m_isXFormation = isXFormation;
  }

  /**
   * Drive the robot using joystick inputs.
   *
   * @param xInput   Speed of the robot in the oblique direction (forwards)
   * @param yInput   Speed of the robot in the lateral direction (sideways)
   * @param rotInput Turn rate of the robot.
   */
  public void drive(double xInput, double yInput, double rotInput) {
    if (m_isXFormation) {
      setModuleStates(kXFormationStates);
    }
    else {
      // Pose2d processedInput = new Pose2d(xInput, yInput, new Rotation2d(rotInput));
      Pose2d processedInput = applyRateLimiting(xInput, yInput, rotInput);

      double xSpeed = processedInput.getX() * m_maxSpeedLin;
      double ySpeed = processedInput.getY() * m_maxSpeedLin;
      double rotSpeed = processedInput.getRotation().getRadians() * m_maxSpeedAng; // fake radians, see #applyRateLimiting

      setModuleStates(
        m_driveKinematics.toSwerveModuleStates(
          m_isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
        )
      );
    }
  }

  private Pose2d applyRateLimiting(double xInput, double yInput, double rotInput) {
    // Basic trig - Cartesian-polar conversion
    double inputMovementMag = Math.hypot(xInput, yInput);
    double inputMovementDir = Math.atan2(yInput, xInput);

    // How fast we can change movement direction
    // - the faster we are moving, the less tight our turns should be to avoid flipping
    double directionSlew = m_currentMovementMag != 0.0
      ? Math.abs(m_slewRateDirection / m_currentMovementMag)
      : 500.0; // magic number found in example code

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDiff = SwerveUtils.AngleDifference(inputMovementDir, m_currentMovementDir);

    if (angleDiff < 0.45 * Math.PI) {
      // Gentle enough turn that we can turn and accelerate at the same time
      m_currentMovementMag = m_limiterMovement.calculate(inputMovementMag);
      m_currentMovementDir = SwerveUtils.StepTowardsCircular(m_currentMovementDir, inputMovementDir, directionSlew * elapsedTime);
    }
    else if (angleDiff > 0.85 * Math.PI) {
      // We are pretty much doing a 180
      if (m_currentMovementMag > 1e-4) {
        // Currently moving, we have to slow down first before changing direction
        m_currentMovementMag = m_limiterMovement.calculate(0.0);
      }
      else {
        // Basically not moving, so we can start to accelerate as well as flip our direction around
        m_currentMovementMag = m_limiterMovement.calculate(inputMovementMag);
        m_currentMovementDir = SwerveUtils.WrapAngle(m_currentMovementDir + Math.PI);
      }
    }
    else {
      // Big turn but not a 180, so we must slow down but can simultaneously change direction
      m_currentMovementMag = m_limiterMovement.calculate(0.0);
      m_currentMovementDir = SwerveUtils.StepTowardsCircular(m_currentMovementDir, inputMovementDir, directionSlew * elapsedTime);
    }

    m_prevTime = currentTime;

    // Bundle up values for returning
    return new Pose2d(
      new Translation2d(m_currentMovementMag, new Rotation2d(m_currentMovementDir)),
      new Rotation2d(m_limiterRotation.calculate(rotInput)) // not actually radians
    );
  }
}