// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE.INTAKE_STATES;

public class Intake extends SubsystemBase {

  private final WPI_TalonSRX intakeMotor1; //right
  private final WPI_TalonSRX intakeMotor2; //left

  private final DoubleSolenoid rightPiston;
  private final DoubleSolenoid leftPiston;

  private INTAKE_STATES intakeState = INTAKE_STATES.IDLE;

  public Intake() {
    intakeMotor1 = new WPI_TalonSRX(8);
    intakeMotor2 = new WPI_TalonSRX(9);
    intakeMotor1.setInverted(false);
    
    intakeMotor2.follow(intakeMotor1);
    intakeMotor2.setInverted(InvertType.OpposeMaster);

    rightPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    leftPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 13);

    SmartDashboard.putBoolean("Subsystems/Intake", false);

  }

  public void setIntakeState(INTAKE_STATES state){
    intakeState = state;

  }

  public void runIntakeSubsystem(){
    SmartDashboard.putBoolean("Subsystems/Intake", true);

    leftPiston.set(intakeState.value);
    rightPiston.set(intakeState.value);

    intakeMotor1.set(intakeState.speed);
  
  }

  public void logIntakeSubsystem(){
    SmartDashboard.putString("Intake/IntakeState", intakeState.name());
  }

}
