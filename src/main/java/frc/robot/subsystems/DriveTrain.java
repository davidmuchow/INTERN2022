// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  // private MotorController leftMotor;
  // private MotorController rightMotor;
  public ArrayList<CANSparkMax[]> SparkMaxArrays;
  DifferentialDrive diffDrive;
  MotorControllerGroup left;
  MotorControllerGroup right;

  public DriveTrain(CANSparkMax[] leftMotors, CANSparkMax[] rightMotors) {
    SparkMaxArrays = new ArrayList<CANSparkMax[]>();
    SparkMaxArrays.add(leftMotors);
    SparkMaxArrays.add(rightMotors);

    left = new MotorControllerGroup(leftMotors[0], leftMotors[1]);
    left.setInverted(true);
    right = new MotorControllerGroup(rightMotors[0], rightMotors[1]);
    diffDrive = new DifferentialDrive(left, right);
  }

  public double line(double input) {
    // 0.1275364 - 2.597472*x + 15.27631*x^2 - 24.70417*x^3 + 16.87782*x^4 - 4.027326*x^5
    return 0.1275364 - 2.597472 * input + 15.27631 * Math.pow(input, 2) - 24.70417 * Math.pow(input, 3) + 16.87782 * Math.pow(input, 4) - 4.027326 * Math.pow(input, 5);
  }



  public void drive(double Y, double Z) {

    SmartDashboard.putNumber("Y:", Y);
    SmartDashboard.putNumber("Z:", Z);
    SmartDashboard.putNumber("Left Motor Output:", SparkMaxArrays.get(0)[0].getAppliedOutput());
    SmartDashboard.putNumber("Right Motor Output:", SparkMaxArrays.get(1)[0].getAppliedOutput());

    //diffDrive.arcadeDrive(line(Y), line(Z));
    diffDrive.arcadeDrive(-Y, -Z);
  }
  public void setMotors(double setpoint) {
    
    diffDrive.tankDrive(setpoint, setpoint);
  }
  public void setCoast() {
    for(CANSparkMax[] motorarr : SparkMaxArrays) {
      for(CANSparkMax currMotor : motorarr) {
        currMotor.setIdleMode(IdleMode.kCoast);
      }
    }
  }

  public void setBrakes() {
    for(CANSparkMax[] motorarr : SparkMaxArrays) {
      for(CANSparkMax currMotor : motorarr) {
        currMotor.setIdleMode(IdleMode.kBrake);
      }
    }
  }
  
  public void rotate(double output) {
      diffDrive.arcadeDrive(0, output);
  }

  @Override
  public void periodic() {}
}
