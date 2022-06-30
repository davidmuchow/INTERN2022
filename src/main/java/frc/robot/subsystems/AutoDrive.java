// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoDrive extends SubsystemBase {
  private DriveTrain driveSub;
  private int CURR_AUTO_MODE = 0;
  private ArrayList<RelativeEncoder> encoders;

  public AutoDrive(DriveTrain driveSub) {
    this.driveSub = driveSub;
    encoders = new ArrayList<RelativeEncoder>();
    for(CANSparkMax[] cur_group : driveSub.SparkMaxArrays) {
      for(CANSparkMax controller_ptr : cur_group) {
        encoders.add(controller_ptr.getEncoder());
      }
    }
  }

  public void useEncoders(double distance) {
    driveSub.setMotors(0.25);
    distance = Units.metersToInches(distance);
    double distanceToRotations = distance / Constants.ENCODER_CONSTATNS.WHEEL_CIRCUMFRENCE;
    double currentTicks = encoders.get(0).getPosition();
    if((currentTicks / Constants.ENCODER_CONSTATNS.TICKS_PER_ROTATION) == distanceToRotations) {
      driveSub.setMotors(0);
    }
  }

  public double useEncoders(double distance, double speed) {
    driveSub.setMotors(speed);
    distance = Units.metersToInches(distance);
    double distanceToRotations = distance / Constants.ENCODER_CONSTATNS.WHEEL_CIRCUMFRENCE;
    double currentTicks = Math.abs(encoders.get(0).getPosition());
    SmartDashboard.putNumber("currentTicks", currentTicks / Constants.ENCODER_CONSTATNS.TICKS_PER_ROTATION);
    SmartDashboard.putNumber("distanceToRotations", distanceToRotations);
    return currentTicks / Constants.ENCODER_CONSTATNS.TICKS_PER_ROTATION;
  }

  public void stopMotors() {
    driveSub.setMotors(0);
  }
  
  public void resetEncoders() {
    for(CANSparkMax[] motorarr : driveSub.SparkMaxArrays) {
      for(CANSparkMax currMotor : motorarr) {
        currMotor.getEncoder().setPosition(0);
      }
    }
  }

  @Override
  public void periodic() {}
}
