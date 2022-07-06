// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  CANSparkMax motorLeftOne = new CANSparkMax(Constants.ID.MOTORLEFT_ONE, MotorType.kBrushless);
  CANSparkMax motorLeftTwo = new CANSparkMax(Constants.ID.MOTORLEFT_TWO, MotorType.kBrushless);
  CANSparkMax motorRightOne = new CANSparkMax(Constants.ID.MOTORRIGHT_ONE, MotorType.kBrushless);
  CANSparkMax motorRightTwo = new CANSparkMax(Constants.ID.MOTORRIGHT_TWO, MotorType.kBrushless);
  static Joystick joy = new Joystick(0);

  public DriveTrain drivey = new DriveTrain(new CANSparkMax[] {motorLeftOne, motorLeftTwo}, new CANSparkMax[] {motorRightOne, motorRightTwo});
  public AutoDrive autoDrive = new AutoDrive(drivey);

  public static AHRS navX = new AHRS();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivey.setDefaultCommand(
      new RunCommand(() -> drivey.drive(joy.getY(), joy.getZ()), drivey)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
