// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmTiltDown;
import frc.robot.commands.Autonomous;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.ClawTiltDown;
import frc.robot.commands.ClawTiltUp;
import frc.robot.commands.SwingingArmDrive;
//import frc.robot.commands.place2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwingingArm;;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Pneumatics m_air = new Pneumatics();
  private final SwingingArm m_arm = new SwingingArm();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  private final Joystick drive_controller = new Joystick(0);
  private final Joystick arm_controller = new Joystick(1);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_chooser.setDefaultOption("Auto Routine Distance", new Autonomous(m_drive));
    SmartDashboard.putData(m_chooser);

    // Drivetrain
    m_drive.setDefaultCommand(
        new ArcadeDrive(m_drive, () -> drive_controller.getY(), () -> drive_controller.getX(),
            () -> drive_controller.getRawAxis(3)));

    // Swinging Arm
    m_arm.setDefaultCommand(new SwingingArmDrive(m_arm, () -> arm_controller.getY()));

    // Claw Open/Close
    JoystickButton b_clawOpen = new JoystickButton(arm_controller, 2);
    JoystickButton b_clawClose = new JoystickButton(arm_controller, 1);

    b_clawOpen.onTrue(new ClawOpen(m_air));
    b_clawClose.onTrue(new ClawClose(m_air));


    // Claw Tilt Up/Down
    JoystickButton b_clawTiltUp = new JoystickButton(arm_controller, 6);
    JoystickButton b_clawTiltDown = new JoystickButton(arm_controller, 4);

    b_clawTiltUp.onTrue(new ClawTiltUp(m_air));
    b_clawTiltDown.onFalse(new ClawTiltDown(m_air));

    // Arm Tilt Up/Down
    JoystickButton b_armTiltUp = new JoystickButton(arm_controller, 3);
    JoystickButton b_armTiltDown = new JoystickButton(arm_controller, 5);

    b_armTiltUp.onTrue(new ArmTiltDown(m_air));
    b_armTiltDown.onTrue(new ArmTiltDown(m_air));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

}