// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwingingArm;
import java.util.function.Supplier;

public class SwingingArmDrive extends CommandBase {
  private final SwingingArm m_arm;
  private final Supplier<Double> m_speed;

  public SwingingArmDrive(SwingingArm arm, Supplier<Double> speed) {
    m_arm = arm;
    m_speed = speed;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.moveArm(m_speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
