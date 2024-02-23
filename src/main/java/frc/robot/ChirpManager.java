// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntUnaryOperator;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;

public class ChirpManager {
  private final ArmSubsystem m_arm;

  private final Orchestra m_orchestra;

  private final String[] m_songs;
  private int m_currentSong = 0;

  public ChirpManager(ArmSubsystem arm, Orchestra orchestra, String... songs) {
    m_arm = arm;
    m_orchestra = orchestra;
    m_songs = songs;

    for (MotorController controller : ((SmartMotorControllerGroup<TalonFX>) m_arm.getPivot()).getControllers()) {
      m_orchestra.addInstrument((TalonFX) controller);
    }

    loadCurrentSong();
  }

  private void loadCurrentSong() {
    m_orchestra.loadMusic(m_songs[m_currentSong] + ".chrp");
  }

  public Command getSongSelectCommand(IntUnaryOperator indexCalculator) {
    return new InstantCommand(
      () -> {
        if (m_orchestra.isPlaying()) {
          m_orchestra.stop();
        }

        m_currentSong = indexCalculator.applyAsInt(m_currentSong);

        if (m_currentSong >= m_songs.length) {
          m_currentSong = 0;
        }
        else if (m_currentSong < 0) {
          m_currentSong = m_songs.length - 1;
        }

        loadCurrentSong();
      },
      m_arm
    );
  }

  public Command getSongPlayCommand(IntUnaryOperator indexCalculator) {
    return new SequentialCommandGroup(
      getSongSelectCommand(indexCalculator),
      getPlayPauseCommand()
    );
  }

  public Command getPlayPauseCommand() {
    return new InstantCommand(
      () -> {
        if (m_orchestra.isPlaying()) {
          m_orchestra.pause();
        } else {
          m_orchestra.play();
        }
      },
      m_arm
    );
  }
}
