// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.StateMachineConstant;
import frc.robot.extensions.StateMachine;
import frc.robot.extensions.StateMachine.BotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cm_SetToDefault extends SequentialCommandGroup {
  /** Creates a new cm_SetToDefault. */
  public cm_SetToDefault(Elevator elevator, LEDSubsystem leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        StateMachine.setGlobalState(BotState.DEFAULT),
        new cm_SetElevatorPosition(elevator).withTimeout(5),
        leds.cm_setLedToColor(StateMachineConstant.botState.colorDisplay));
  }
}
