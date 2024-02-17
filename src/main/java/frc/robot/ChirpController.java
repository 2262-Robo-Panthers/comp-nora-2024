// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;

public class ChirpController {
  private final ArmSubsystem m_arm;

  private final Orchestra m_orchestra = new Orchestra();

  private final String[] m_songs;
  private int m_currentSong = 0;

  public ChirpController(ArmSubsystem arm, String... songs) {
    m_arm = arm;
    m_songs = songs;

    for (MotorController controller : ((SmartMotorControllerGroup<TalonFX>) m_arm.getPivot()).getControllers()) {
      m_orchestra.addInstrument((TalonFX) controller);
    }
  }

  public Command getNextSongCommand() {
    return new InstantCommand(
      () -> {
        if (m_orchestra.isPlaying()) {
          m_orchestra.stop();
        }

        m_orchestra.loadMusic(m_songs[m_currentSong++] + ".chrp");
        m_orchestra.play();

        if (m_currentSong >= m_songs.length) {
          m_currentSong = 0;
        }
      },
      m_arm
    );
  }

  public Command getStopCommand() {
    return new InstantCommand(m_orchestra::stop, m_arm);
  }
}
