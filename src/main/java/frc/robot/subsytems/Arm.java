// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.RATCHET_STATES;
import frc.robot.Constants.PivotConstants.SETPOINTS;
import frc.robot.Constants.PivotConstants.WINCH_STATES;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX pivotMotor;
  private final WPI_TalonSRX winchMotor;
  private final DoubleSolenoid pivotRatchet;
  private final DutyCycleEncoder pivotEncoder;
  private final PIDController pivotPID;

  public SETPOINTS target = Constants.PivotConstants.SETPOINTS.START;
  public double falconOffset;
  public double trimVal = 0;

  public WINCH_STATES winchState = WINCH_STATES.IDLE;

  public boolean firstRun = true;

  public Arm() {

    pivotMotor = new WPI_TalonFX(Constants.CAN.kPivot);
    winchMotor = new WPI_TalonSRX(Constants.CAN.kArm);
    pivotRatchet = new DoubleSolenoid(7, PneumaticsModuleType.REVPH, 6, 7);
    pivotEncoder = new DutyCycleEncoder(9);
    pivotPID = new PIDController(2.5 / 3, 0, 0);

    pivotMotor.setInverted(true);
    pivotMotor.setSelectedSensorPosition(0);

    pivotEncoder.setPositionOffset(Constants.PivotConstants.kThroughboreOffset);
    falconOffset = degreeToFalcon(getThroughBoreAngle());
    pivotPID.setTolerance(3, 7);

    pivotMotor.configFactoryDefault();
    pivotMotor.setNeutralMode(NeutralMode.Brake);
    pivotMotor.configVoltageCompSaturation(10);
    pivotMotor.enableVoltageCompensation(true);
    pivotMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));

    winchMotor.configFactoryDefault();
    winchMotor.setNeutralMode(NeutralMode.Brake);
    winchMotor.configVoltageCompSaturation(10);
    winchMotor.enableVoltageCompensation(true);
    winchMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));

    SmartDashboard.putBoolean("Subsystems/Arm", false);
    
  }

  private void runPivot() {
    if (atTarget()) {
      pivotMotor.set(0);
      setRatchetState(RATCHET_STATES.ENGAGED);
      firstRun = true;

    } else {
      setRatchetState(RATCHET_STATES.DISENGAGED);

      if (firstRun) {
        Timer.delay(0.1);
      }

      double mPIDEffort = pivotPID.calculate(
          getAngle(),
          MathUtil.clamp(target.angle + trimVal, -20, 80));

      pivotMotor.set(mPIDEffort / 12);

      firstRun = false;
    }
  }

  private void runWinch(){
    winchMotor.set(winchState.value);
  }

  public boolean atTarget() {

    double tolerance = 2;

    return (Math.abs(getAngle() - target.angle + trimVal) < tolerance);

  }

  public void setPivotState(PivotConstants.SETPOINTS setpoint) {
    target = setpoint;
  }

  public void setWinchState(WINCH_STATES state){
    winchState = state;
  }

  public void setRatchetState(RATCHET_STATES state) {
    pivotRatchet.set(state.value);
  }

  public double getThroughBoreAngle() {
    return ((pivotEncoder.getAbsolutePosition()) - pivotEncoder.getPositionOffset()) * 360;
  }

  public double degreeToFalcon(double deg) {
    return deg * (1d / 360d) * (60d / 16d) * 100 * 2048;
  }

  public double falconToDegrees(double val) {
    return val * 1 / 2048d * 1 / 100 * 16 / 60d * 360;
  }

  public double getAngle() {
    return falconToDegrees(pivotMotor.getSelectedSensorPosition() + falconOffset);
  }

  public void runArmSubsystem(){
    SmartDashboard.putBoolean("Subsystems/Arm", true);
    runPivot();
    runWinch();
  }

  public void logArmSubsystem(){
    SmartDashboard.putString("Arm/Setpoint" , target.name());
    SmartDashboard.putNumber("Arm/Setpoint Angle", target.angle);
    SmartDashboard.putNumber("Arm/Throughbore Angle", getThroughBoreAngle());
    SmartDashboard.putNumber("Arm/Falcon Angle", getAngle());
    SmartDashboard.putBoolean("Arm/At Setpoint", atTarget());
    SmartDashboard.putString("Arm/Winch State", winchState.name());
  }

}
