// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsytems.Arm;
import frc.robot.subsytems.Drivetrain;
import frc.robot.subsytems.Intake;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Drivetrain drivetrain;
  private Arm arm;
  private Intake intake;

  private CommandXboxController driverController = new CommandXboxController(0);

  @Override
  public void robotInit() {
   
    drivetrain = new Drivetrain();
    arm = new Arm();
    intake = new Intake();

    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.drive(
          driverController.getRightX(), //rotate
          driverController.getLeftX(), //strafe
          driverController.getLeftY(), //forward
          false 
        ), 
      drivetrain)
    );

    driverController.start().onTrue(
      new InstantCommand(
        drivetrain::resetGyro, drivetrain
      )
    );

    driverController.rightTrigger().onTrue(
      new InstantCommand(
        () -> intake.setIntakeState(Constants.INTAKE.INTAKE_STATES.OUTTAKING),
        intake
      )
    ).onFalse(
      new InstantCommand(
        () -> intake.setIntakeState(Constants.INTAKE.INTAKE_STATES.IDLE),
        intake
      )
    );

    driverController.leftTrigger().onTrue(
      new InstantCommand(
        () -> intake.setIntakeState(Constants.INTAKE.INTAKE_STATES.INTAKING),
        intake
      )
    ).onFalse(
      new InstantCommand(
        () -> intake.setIntakeState(Constants.INTAKE.INTAKE_STATES.IDLE),
        intake
      )
    );

    driverController.a().onTrue(
      new InstantCommand(
        () -> arm.setPivotState(Constants.PivotConstants.SETPOINTS.INTAKE),
        arm
      )
    );

    driverController.b().onTrue(
      new InstantCommand(
        () -> arm.setPivotState(Constants.PivotConstants.SETPOINTS.ZERO),
        arm
      )
    );

    driverController.x().onTrue(
      new InstantCommand(
        () -> arm.setPivotState(Constants.PivotConstants.SETPOINTS.START),
        arm
      )
    );

    driverController.y().onTrue(
      new InstantCommand(
        () -> arm.setPivotState(Constants.PivotConstants.SETPOINTS.CARRY),
        arm
      )
    );
  
    driverController.povUp().onTrue(
      new InstantCommand(
        () -> arm.setWinchState(Constants.PivotConstants.WINCH_STATES.EXTENDING),
        arm
      )
    ).onFalse(
      new InstantCommand(
        () -> arm.setWinchState(Constants.PivotConstants.WINCH_STATES.IDLE),
        arm
      )
    );

    driverController.povDown().onTrue(
      new InstantCommand(
        () -> arm.setWinchState(Constants.PivotConstants.WINCH_STATES.RETRACTING),
        arm
      )
    ).onFalse(
      new InstantCommand(
        () -> arm.setWinchState(Constants.PivotConstants.WINCH_STATES.IDLE),
        arm
      )
    );

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    drivetrain.logDrivetrain();
    arm.logArmSubsystem();
    intake.logIntakeSubsystem();

  }

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    arm.runArmSubsystem();
    intake.runIntakeSubsystem();

  }

}
