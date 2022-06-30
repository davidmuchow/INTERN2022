// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnWithGyro extends PIDCommand {
  /** Creates a new TurnWithGyro. */
  AHRS navX;
  public TurnWithGyro(AHRS navX, double setpoint, DriveTrain driveSub) {
    super(
        // The controller that the command will use
        new PIDController(0.12, 0, 0),
        // This should return the measurement
        navX::getAngle,
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          if(setpoint > 0) {
            driveSub.rotate(output);
          }
          else {
            driveSub.rotate(-output);
          }
        });
      navX.reset();
      this.navX = navX;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveSub);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(navX.getAngle() -  getController().getSetpoint()) <= 2);
  }
}
