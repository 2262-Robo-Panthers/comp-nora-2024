// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.UnaryOperator;

import edu.wpi.first.networktables.Topic;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.util.ShuffleboardTabWithMaps;

public class ChirpManager {
  private final ArmSubsystem m_arm;

  private final Orchestra m_orchestra;

  private final String m_name;
  private final String[] m_songs;
  private int m_currentSong = 0;

  private boolean m_wasInterrupted = false;

  @SuppressWarnings("unchecked")
  public ChirpManager(ShuffleboardTab shuffleboardTab, ArmSubsystem arm, Orchestra orchestra, String name, String... songs) {
    m_arm = arm;
    m_orchestra = orchestra;
    m_name = name;
    m_songs = songs;

    for (MotorController controller : ((SmartMotorControllerGroup<TalonFX>) m_arm.getPivot()).getControllers()) {
      m_orchestra.addInstrument((TalonFX) controller);
    }

    loadCurrentSong();

    populateDashboard(shuffleboardTab);
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    Topic isEnabled =
    ShuffleboardTabWithMaps.addMap(dashboard, "ChirpManager." + m_name, "%s", Map.of(
      "State", () -> m_orchestra.isPlaying() ? "playing" : "paused",
      "CurrentSong", () -> m_songs[m_currentSong]
    )).add(
      "IsEnabled", false
    )
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
      .getTopic();

    new NetworkButton(new BooleanTopic(isEnabled))
      .onFalse(new InstantCommand(() -> {
        if (m_orchestra.isPlaying()) {
          m_wasInterrupted = true;
          m_orchestra.stop();
        }
      }, m_arm))
      .onTrue(new InstantCommand(() -> {
        if (m_wasInterrupted) {
          m_orchestra.play();
          m_wasInterrupted = false;
        }
      }, m_arm));
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
      m_arm
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
        } else {
          m_orchestra.play();
        }
      },
      m_arm
    );
  }
}
