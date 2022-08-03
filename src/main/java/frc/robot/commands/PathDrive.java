// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.cert.TrustAnchor;

import javax.swing.plaf.ProgressBarUI;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.AutoDrive;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

public class PathDrive extends CommandBase {
  /** Creates a new PathDrive. */

  private DriveTrain driveSub;
  private AutoDrive autoDriveSub;

  public PathDrive(DriveTrain driveSub, AutoDrive autoDriveSub) {
    this.driveSub = driveSub;
    this.autoDriveSub = autoDriveSub;
    addRequirements(driveSub);
  }

  public Command profile1() {
    return new SequentialCommandGroup(
      new EncoderDriveDistance(autoDriveSub, driveSub, 3, 0.3),
      new TurnWithGyro(180, driveSub, RobotContainer.navX),
      new EncoderDriveDistance(autoDriveSub, driveSub, 3, 0.3)
    );
  }

  public Command square() {
    return new SequentialCommandGroup(
      new EncoderDriveDistance(autoDriveSub, driveSub, 3, 0.15),
      new TurnWithGyro(83, driveSub, RobotContainer.navX),
      new EncoderDriveDistance(autoDriveSub, driveSub, 1, 0.15),
      new TurnWithGyro(83, driveSub, RobotContainer.navX),
      new EncoderDriveDistance(autoDriveSub, driveSub, 3, 0.15),
      new TurnWithGyro(83, driveSub, RobotContainer.navX),
      new EncoderDriveDistance(autoDriveSub, driveSub, 1, 0.15),
      new TurnWithGyro(83, driveSub, RobotContainer.navX)
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    square().schedule();
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
