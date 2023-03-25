// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.System;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Clamp.SetClampCube;
import frc.robot.commands.Clamp.SetClampMid;
import frc.robot.commands.Clamp.SetClampTop;
import frc.robot.commands.Clamp.SetClampTopHold;
import frc.robot.commands.Elevator.ElevatorBottom;
import frc.robot.commands.Elevator.ElevatorDownMid;
import frc.robot.commands.Elevator.ElevatorHigh;
import frc.robot.commands.Elevator.ElevatorUpMid;
import frc.robot.commands.Elevator.TiltElevatorDown;
import frc.robot.commands.Elevator.TiltElevatorUp;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoResetToBottom extends SequentialCommandGroup {
  /** Creates a new PlaceHigh. */
  private Elevator elevator = Elevator.getInstance();
  private Clamp clamp = Clamp.getInstance();
  public AutoResetToBottom() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClampTop(clamp),
      new ElevatorDownMid(elevator),
      //new InstantCommand(() -> elevator.tiltElevatorDown()),
      //new WaitCommand(1.5),
      new SetClampCube(clamp),
      new ElevatorBottom(elevator)
    );
  }
}
