// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonSRX frontLeft;
  private final WPI_TalonSRX frontRight;
  private final WPI_TalonSRX backLeft;
  private final WPI_TalonSRX backRight;

  private final WPI_PigeonIMU gyro;

  private final MecanumDrive drive;

  public Drivetrain() {
    frontLeft = new WPI_TalonSRX(1);
    frontRight = new WPI_TalonSRX(2);
    backLeft = new WPI_TalonSRX(3);
    backRight = new WPI_TalonSRX(4);

    gyro = new WPI_PigeonIMU(6);

    drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 1);

    frontLeft.configSupplyCurrentLimit(currentLimit);
    frontRight.configSupplyCurrentLimit(currentLimit);
    backLeft.configSupplyCurrentLimit(currentLimit);
    backRight.configSupplyCurrentLimit(currentLimit);

    SmartDashboard.putBoolean("Subsystems/Drivetrain", false);

  }

  public void drive(double _xInput, double _yInput, double _rInput, boolean _fieldRelative){

    SmartDashboard.putBoolean("Subsystems/Drivetrain", true);

    _xInput = MathUtil.clamp(_xInput, -1, 1);
    _yInput = MathUtil.clamp(_yInput, -1, 1);
    _rInput = MathUtil.clamp(_rInput, -1, 1);

    _xInput = MathUtil.applyDeadband(_xInput, 0.1);
    _yInput = MathUtil.applyDeadband(_yInput, 0.1);
    _rInput = MathUtil.applyDeadband(_rInput, 0.1);
    
    if(_fieldRelative){
      drive.driveCartesian(_xInput, _yInput, _rInput, gyro.getRotation2d());
    } else {
      drive.driveCartesian(_xInput, _yInput, _rInput);
    }
    
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void logDrivetrain(){
    SmartDashboard.putNumber("Drivetrain/FL Set Value", frontLeft.get());
    SmartDashboard.putNumber("Drivetrain/FR Set Value", frontRight.get());
    SmartDashboard.putNumber("Drivetrain/BL Set Value", backLeft.get());
    SmartDashboard.putNumber("Drivetrain/BR Set Value", backRight.get());
    SmartDashboard.putNumber("Drivetrain/Gryo Angle", gyro.getFusedHeading());
  }

}
