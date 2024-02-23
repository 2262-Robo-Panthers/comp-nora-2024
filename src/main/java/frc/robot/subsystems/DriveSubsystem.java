// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.MAXSwerve.MAXSwerveModule;

public class DriveSubsystem extends SubsystemBase {
  public final MAXSwerveModule[] m_modules;

  public DriveSubsystem(MAXSwerveModule... modules) {
    m_modules = modules;
  }

  public void zeroVelocity() {
    for (MAXSwerveModule module : m_modules) {
      module.freeze();
    }
  }

  public void setDesiredStates_TEMP(SwerveModuleState desiredState) {
    for (MAXSwerveModule module : m_modules) {
      module.setDesiredState(desiredState);
    }
  }
}