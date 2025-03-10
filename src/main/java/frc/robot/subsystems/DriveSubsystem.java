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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import static edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kZ;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.WPIUtilJNI;

import frc.robot.lib.MAXSwerve.MAXSwerveModule;
import frc.robot.lib.MAXSwerve.SwerveUtils;
import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants;

public class DriveSubsystem extends SubsystemBase {
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

  private GenericEntry m_isXFormation;
  private GenericEntry m_isFieldRelative;

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
    m_maxSpeedLin = maxSpeedLin_m_s;
    m_maxSpeedAng = maxSpeedAng_rad_s;
    m_isGyroReversed = isGyroReversed;

    m_limiterMovement = new SlewRateLimiter(slewRateMovement);
    m_limiterRotation = new SlewRateLimiter(slewRateRotation);
    m_slewRateDirection = slewRateDirection;

    m_modules = modules;
    m_odometry = new SwerveDriveOdometry(driveKinematics, getRotation(), getModulePositions());
    m_driveKinematics = driveKinematics;

    populateDashboard(shuffleboardTab);
  }

  @Override
  public void periodic() {
    m_odometry.update(getRotation(), getModulePositions());
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, ShuffleboardConstants.PoseInfo, this::getPose, List.of(
      new Pair<>("X Position", new Pair<>("%.3f m", pose -> pose.getX())),
      new Pair<>("Y Position", new Pair<>("%.3f m", pose -> pose.getY()))
    ))
      .addDouble("Orientation", () -> 0.0 - getPose().getRotation().getDegrees()) // Negative because Gyro widget is CW+
      .withWidget(BuiltInWidgets.kGyro);

    ShuffleboardLayout driveInfo =
    ShuffleboardTabWithMaps.addMap(dashboard, ShuffleboardConstants.DriveInfo, List.of(
      new Pair<>("Gyro Angle", new Pair<>("%.3f\u00b0", this::getRotation_deg)),
      new Pair<>("Gyro Spin", new Pair<>("%.3f\u00b0/s", this::getRotationRate_deg_s))
    ));

    m_isXFormation =
    driveInfo.add("X Fmtion.", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

    m_isFieldRelative =
    driveInfo.add("Field Rel.", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void usePose(Pose2d pose) {
    m_odometry.resetPosition(getRotation(), getModulePositions(), pose);
  }

  public void usePoseTranslation(Translation2d translation) {
    usePose(new Pose2d(translation, m_odometry.getPoseMeters().getRotation()));
  }

  public void usePoseTranslationX(double x) {
    usePoseTranslation(new Translation2d(x, m_odometry.getPoseMeters().getY()));
  }

  public void usePoseTranslationY(double y) {
    usePoseTranslation(new Translation2d(m_odometry.getPoseMeters().getX(), y));
  }

  public void usePoseRotation(Rotation2d rotation) {
    m_gyro.setGyroAngle(kZ, rotation.getRadians());
    usePose(new Pose2d(m_odometry.getPoseMeters().getTranslation(), rotation));
  }

  public void useCurrentTranslationAsOrigin() {
    usePoseTranslation(new Translation2d(0.0, 0.0));
  }

  public void useCurrentRotationAsOrigin() {
    usePoseRotation(new Rotation2d(0.0));
  }

  public void useCurrentPoseAsOrigin() {
    useCurrentTranslationAsOrigin();
    useCurrentRotationAsOrigin();
  }

  public double getRotation_deg() {
    return getRotation().getDegrees();
  }

  public double getRotationRate_deg_s() {
    return m_gyro.getRate(kZ) * (m_isGyroReversed ? -1.0 : 1.0);
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
    m_isFieldRelative.setBoolean(isFieldRelative);
  }

  public void setXFormation(boolean isXFormation) {
    m_isXFormation.setBoolean(isXFormation);
  }

  /**
   * Drive the robot using joystick inputs.
   *
   * @param xInput   Speed of the robot in the oblique direction (forwards)
   * @param yInput   Speed of the robot in the lateral direction (sideways)
   * @param rotInput Turn rate of the robot.
   */
  public void drive(double xInput, double yInput, double rotInput) {
    if (m_isXFormation.getBoolean(false)) {
      setModuleStates(kXFormationStates);
    }
    else {
      Pose2d processedInput = applyRateLimiting(xInput, yInput, rotInput);

      double xSpeed = processedInput.getX() * m_maxSpeedLin;
      double ySpeed = processedInput.getY() * m_maxSpeedLin;
      double rotSpeed = processedInput.getRotation().getRadians() * m_maxSpeedAng; // fake radians, see #applyRateLimiting

      setModuleStates(
        m_driveKinematics.toSwerveModuleStates(
          m_isFieldRelative.getBoolean(false)
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