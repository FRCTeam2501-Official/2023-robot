// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;



public class ClawOpen extends CommandBase {
  /** Creates a new clawopen. */
  
  public final Pneumatics m_air;
  
  public ClawOpen(Pneumatics air) {
    m_air = air;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(air);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_air.claw(1);
    //m_air.compressorswich(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
