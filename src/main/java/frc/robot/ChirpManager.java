// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants.CardMetadata;

public class ChirpManager {
  private final ShoulderSubsystem m_shoulder;

  private final Orchestra m_orchestra;

  private final String[] m_songs;
  private int m_currentSong = 0;

  private boolean m_isEnabled = false;
  private boolean m_wasInterrupted = false;

  public ChirpManager(ShuffleboardTab shuffleboardTab, ShoulderSubsystem shoulder, Orchestra orchestra, CardMetadata metadata, String... songs) {
    m_shoulder = shoulder;
    m_orchestra = orchestra;
    m_songs = songs;

    for (TalonFX controller : shoulder.getControllers()) {
      m_orchestra.addInstrument(controller);
    }

    loadCurrentSong();

    populateDashboard(shuffleboardTab, metadata);
  }

  private void populateDashboard(ShuffleboardTab dashboard, CardMetadata metadata) {
    Topic isEnabled =
    ShuffleboardTabWithMaps.addMap(dashboard, metadata, "%s", List.of(
      new Pair<>("State", () -> m_orchestra.isPlaying() ? "playing" : "paused"),
      new Pair<>("Loaded", () -> m_songs[m_currentSong].split("/")[1])
    ))
      .add("Enabled?", m_isEnabled)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry()
      .getTopic();

    new NetworkButton(new BooleanTopic(isEnabled))
      .onFalse(Commands.runOnce(() -> {
        m_isEnabled = false;
        if (m_orchestra.isPlaying()) {
          m_orchestra.pause();
          m_wasInterrupted = true;
        }
      }, m_shoulder))
      .onTrue(Commands.runOnce(() -> {
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
    return Commands.runOnce(() -> {
      if (m_orchestra.isPlaying())
        m_orchestra.stop();

      m_currentSong = indexCalculator.apply(m_currentSong);

      if (m_currentSong >= m_songs.length)
        m_currentSong = 0;
      else if (m_currentSong < 0)
        m_currentSong = m_songs.length - 1;

      loadCurrentSong();
    }, m_shoulder);
  }

  public Command getSongPlayCommand(UnaryOperator<Integer> indexCalculator) {
    return
      getSongSelectCommand(indexCalculator)
      .andThen(
      getPlayPauseCommand());
  }

  public Command getPlayPauseCommand() {
    return Commands.runOnce(() -> {
      if (m_orchestra.isPlaying())
        m_orchestra.pause();
      else if (m_isEnabled)
        m_orchestra.play();
    }, m_shoulder);
  }
}
