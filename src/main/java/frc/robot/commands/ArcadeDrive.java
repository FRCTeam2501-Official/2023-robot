// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private final Drivetrain m_drive;
  private final Supplier<Double> m_turn;
  private final Supplier<Double> m_speed;
  private Supplier<Double> m_slow;
  private double Slow;
  private double slowturn;

  public ArcadeDrive(Drivetrain drive, Supplier<Double> speed, Supplier<Double> turn, Supplier<Double> slow) {
    // Use addRequirements() here to declare subsystem dependencies.,
    m_drive = drive;
    m_turn = turn;
    m_speed = speed;
    m_slow = slow;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    Slow = (m_slow.get() + 1) * (1 - .25) / (1 + 1) + .25;
    slowturn = (m_slow.get() + 1) * (1 - .75) / (1 + 1) + .75;
    m_drive.arcadedrive(m_speed.get() * Slow, m_turn.get() * slowturn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadedrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
