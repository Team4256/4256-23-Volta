// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.System;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Clamp.SetClampFeederStation;
import frc.robot.commands.Clamp.SetClampTop;
import frc.robot.commands.Clamp.SetClampTopHold;
import frc.robot.commands.Elevator.ElevatorConeMid;
import frc.robot.commands.Elevator.ElevatorFeederStation;
import frc.robot.commands.Elevator.ElevatorHigh;
import frc.robot.commands.Elevator.ElevatorUpMid;
import frc.robot.commands.Elevator.TiltElevatorDown;
import frc.robot.commands.Elevator.TiltElevatorUp;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeMidPosition extends SequentialCommandGroup {
  /** Creates a new PlaceHigh. */
  private Elevator elevator = Elevator.getInstance();
  private Clamp clamp = Clamp.getInstance();
  public ConeMidPosition() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorUpMid(elevator),
      new SetClampTop(clamp),
      new InstantCommand(() -> elevator.tiltElevatorUp()),
      new WaitCommand(1),
      new ElevatorConeMid(elevator),
      new SetClampTop(clamp)
    );
  }
}
