// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.WinchSubsystem;
import frc.robot.util.FunctionalUtil;
import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants;

public class WinchCommand extends Command {
  private final WinchSubsystem m_winch;

  private final Supplier<Double> m_climb;

  public WinchCommand(
    ShuffleboardTab shuffleboardTab,
    WinchSubsystem winch,
    Supplier<Double> climb,
    double deadband
  ) {
    m_winch = winch;

    UnaryOperator<Double> deadbandify = FunctionalUtil.deadbandify(deadband);
    m_climb = FunctionalUtil.supplyThenOperate(climb, deadbandify);

    populateDashboard(shuffleboardTab);

    addRequirements(winch);
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, ShuffleboardConstants.ArmInfo, "%.3f", List.of(
      new Pair<>("Climb", m_climb)
    ));
  }

  @Override
  public void initialize() {
    m_winch.stop();
  }

  @Override
  public void execute() {
    m_winch.setClimbSpeed(m_climb.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_winch.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
