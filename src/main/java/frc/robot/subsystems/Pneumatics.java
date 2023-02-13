// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html

public class Pneumatics extends SubsystemBase {
  /** Creates a new pneumatics. */
  Compressor m_compressor = new Compressor(22, PneumaticsModuleType.CTREPCM);
  // Compressor m_compressor = new Compressor(22, PneumaticsModuleType.CTREPCM);
  // Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final DoubleSolenoid m_claw = new DoubleSolenoid(22, PneumaticsModuleType.CTREPCM, 0, 1);
  private final DoubleSolenoid m_wrist = new DoubleSolenoid(22, PneumaticsModuleType.CTREPCM, 2, 3);
  private final DoubleSolenoid m_tilt = new DoubleSolenoid(22, PneumaticsModuleType.CTREPCM, 4, 5);
  // 3 air 1 motor
  private final PIDController m_pivitepid = new PIDController(0, 0, 0);
  // private final CANSparkMax m_pivetmotor = new CANSparkMax(10,
  // MotorType.kBrushless);

  private final DigitalInput limit = new DigitalInput(0);

  private double zero = 0;

  public Pneumatics() {
    // m_pivetmotor.getPIDController();
  }

  // pass a value of 1 to open the claw, a -1 to close the claw, or a 0 to disable
  // the claw
  public void wrist(double pole) {
    if (pole == 1) {
      m_wrist.set(Value.kForward);
    }
    if (pole == 0) {
      m_wrist.set(Value.kOff);
    }
    if (pole == -1) {
      m_wrist.set(Value.kReverse);
    }
  }

  public void claw(double pole2) {
    if (pole2 == 1) {
      m_claw.set(Value.kForward);
    }
    if (pole2 == 0) {
      m_claw.set(Value.kOff);
    }
    if (pole2 == -1) {
      m_claw.set(Value.kReverse);
    }
  }

  public void tilt(double pole3) {
    if (pole3 == 1) {
      m_tilt.set(Value.kForward);
    }
    if (pole3 == 0) {
      m_tilt.set(Value.kOff);
    }
    if (pole3 == -1) {
      m_tilt.set(Value.kReverse);
    }
  }

  public void pivit(double angale, double power) {
    // m_pivitepid.setSetpoint(angale);
  }

  public double encoder() {
    return 1;// ((RelativeEncoder) m_pivetmotor).getPosition() / 100 - zero;
  }

  public boolean limit() {
    return limit.get();
  }

  public void zeroencoderhome() {
    // zero = ((RelativeEncoder) m_pivetmotor).getPosition();
  }

  public void zeroencoderpickup1() {
    // zero = ((RelativeEncoder) m_pivetmotor).getPosition() + 10;
  }

  // pass a true to enable the compressor or a false to disable the compressor
  public void compressorswich(boolean swich) {
    if (swich == true) {
      // m_compressor.enabled();
    }
    if (swich == false) {
      // m_compressor.disable();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
