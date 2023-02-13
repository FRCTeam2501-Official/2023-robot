// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // define
  // motors
  // https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json.
  private final CANSparkMax m_motorfl = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_motorfr = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax m_motorbl = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_motorbr = new CANSparkMax(2, MotorType.kBrushless);
  // MotorControllerGroups to controll 2 or motors at the same time
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_motorbr, m_motorfr);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_motorbl, m_motorfl);
  // DifferentialDrive for arcade drive
  private final DifferentialDrive m_diff = new DifferentialDrive(m_left, m_right);
  // varibls for encoders
  private double m_fre;
  private double m_ble;
  private double m_fle;
  private double m_bre;
  private double m_fre0 = 0;
  private double m_ble0 = 0;
  private double m_fle0 = 0;
  private double m_bre0 = 0;
  // gyro
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);

  public Drivetrain() {
    // setup
    m_left.setInverted(true);

  }

  // pass to varibols bolth between -1 and 1 the first one controls speed and the
  // secent one controls turning
  public void arcadedrive(double speed, double turn) {
    m_diff.arcadeDrive(speed, turn);
  }
  // will return the encoder value

  // will 0 the encoders
  public void zeroencoders() {
    m_fre0 = m_fre;
    m_fle0 = m_fle;
    m_bre0 = m_bre;
    m_ble0 = m_ble;
  }
  // will return the raw z gyro angle

  public double getgyroz() {
    return m_gyro.getYaw();
  }

  // will return the z gyro angle from -180 to 180
  public double getGyroAngleZ180() {
    double angle;
    angle = m_gyro.getYaw();
    angle %= 360;
    if (angle > 180) {
      angle -= 360;
    }
    if (angle < -180) {
      angle += 180;
    }
    return angle;
  }

  // will return the raw x gyro angle
  public double getgyrox() {
    return m_gyro.getPitch();
  }

  // will zero the gyro
  public void zerogyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
