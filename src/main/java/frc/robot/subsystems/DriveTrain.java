// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  // private MotorController leftMotor;
  // private MotorController rightMotor;
  DifferentialDrive diffDrive;
  private double joystickXInput;
  private double joystickZInput;

  public static Joystick joysitck;

  public DriveTrain(MotorController leftMotor, MotorController rightMotor) {
    diffDrive = new DifferentialDrive(leftMotor, rightMotor);
  }
  public void Drive() {
    diffDrive.arcadeDrive(joystickXInput, joystickZInput);
  }

  @Override
  public void periodic() {
    joystickXInput = DriveTrain.joysitck.getX();
    joystickZInput = DriveTrain.joysitck.getZ();
  }
}
