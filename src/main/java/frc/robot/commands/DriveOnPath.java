// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.AUTO_CONSTANTS;

public class DriveOnPath extends CommandBase {
  /** Creates a new DriveOnPath. */
  public PathPlannerTrajectory traj;
  public RamseteCommand command;
  public AutoDrive auto;
  public DriveTrain drive;

  public DriveOnPath(PathPlannerTrajectory traj, AutoDrive auto, DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.traj = traj;
    this.auto = auto;
    this.drive = drive;
    addRequirements(auto, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    auto.resetOdometry(traj.getInitialPose());
    this.command = new RamseteCommand(
      traj, 
      auto::getPose, 
      new RamseteController(AUTO_CONSTANTS.kRamseteB, AUTO_CONSTANTS.kRamseteZeta), 
      new SimpleMotorFeedforward(AUTO_CONSTANTS.ksVolts, AUTO_CONSTANTS.kvVoltSecondsPerMeter, AUTO_CONSTANTS.kaVoltSecondsSquaredPerMeter),
      AUTO_CONSTANTS.kDriveKinematics, 
      auto::getWheelSpeeds, 
      new PIDController(AUTO_CONSTANTS.kPDriveVel, 0, 0),
      new PIDController(AUTO_CONSTANTS.kPDriveVel, 0, 0),
      auto::tankDriveVolts,
      auto,
      drive
    );
    command.schedule();
    
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
    return command.isFinished();
  }
}
