// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.UnaryOperator;

import edu.wpi.first.networktables.Topic;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.util.ShuffleboardHelper;
import frc.robot.util.ShuffleboardHelper.DataPoint;

public class ChirpManager {
  private final ShoulderSubsystem m_shoulder;

  private final Orchestra m_orchestra;

  private final String m_name;
  private final String[] m_songs;
  private int m_currentSong = 0;

  private boolean m_isEnabled = false;
  private boolean m_wasInterrupted = false;

  public ChirpManager(ShuffleboardTab shuffleboardTab, ShoulderSubsystem shoulder, Orchestra orchestra, String name, int x, int y, String... songs) {
    m_shoulder = shoulder;
    m_orchestra = orchestra;
    m_name = name;
    m_songs = songs;

    for (TalonFX controller : m_shoulder.getControllers()) {
      m_orchestra.addInstrument(controller);
    }

    loadCurrentSong();

    populateDashboard(shuffleboardTab, x, y);
  }

  private void populateDashboard(ShuffleboardTab dashboard, int x, int y) {
    Topic isEnabled =
    ShuffleboardHelper.add(dashboard, "ChirpManager." + m_name, List.of(
      DataPoint.ofBoolean("IsPlaying", m_orchestra::isPlaying),
      DataPoint.ofString("CurrentSong", () -> m_songs[m_currentSong])
    ))
      .withPosition(x, y)
      .withSize(2, 2)
      .add("IsEnabled", m_isEnabled)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
      .getTopic();

    new NetworkButton(new BooleanTopic(isEnabled))
      .onFalse(new InstantCommand(() -> {
        m_isEnabled = false;
        if (m_orchestra.isPlaying()) {
          m_orchestra.pause();
          m_wasInterrupted = true;
        }
      }, m_shoulder))
      .onTrue(new InstantCommand(() -> {
        m_isEnabled = true;
        if (m_wasInterrupted) {
          m_orchestra.play();
          m_wasInterrupted = false;
        }
      }, m_shoulder));
  }

  private void loadCurrentSong() {
    m_orchestra.loadMusic(m_songs[m_currentSong] + ".chrp");
  }

  public Command getSongSelectCommand(UnaryOperator<Integer> indexCalculator) {
    return new InstantCommand(
      () -> {
        if (m_orchestra.isPlaying()) {
          m_orchestra.stop();
        }

        m_currentSong = indexCalculator.apply(m_currentSong);

        if (m_currentSong >= m_songs.length) {
          m_currentSong = 0;
        }
        else if (m_currentSong < 0) {
          m_currentSong = m_songs.length - 1;
        }

        loadCurrentSong();
      },
      m_shoulder
    );
  }

  public Command getSongPlayCommand(UnaryOperator<Integer> indexCalculator) {
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
        } else if (m_isEnabled) {
          m_orchestra.play();
        }
      },
      m_shoulder
    );
  }
}
