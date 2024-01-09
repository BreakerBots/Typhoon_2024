// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.bindings;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class BreakerButtonBinding {
    public enum BreakerButtonBindingType {
        ON_TRUE,
        ON_FALSE,
        WHILE_TRUE,
        WHILE_FALSE,
        TOGGLE_ON_TRUE,
        TOGGLE_ON_FALSE
    }

    private Trigger bindingTrigger;
    private BreakerButtonBindingType bindingType;
    private Command commandToBind;

    public BreakerButtonBinding(Trigger bindingTrigger, BreakerButtonBindingType bindingType, Command commandToBind) {
        this.bindingTrigger = bindingTrigger;
        this.bindingType = bindingType;
        this.commandToBind = commandToBind;
    }


    public void bind() {
       bindConditionaly(() -> true);
    }

    
    /** 
     * @param isBindingEnabledSupplier
     */
    public void bindConditionaly(BooleanSupplier isBindingEnabledSupplier) {
        switch (bindingType) {
            case ON_FALSE:
                bindingTrigger.onFalse(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            case ON_TRUE:
                bindingTrigger.onTrue(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            case TOGGLE_ON_FALSE:
                bindingTrigger.toggleOnFalse(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            case TOGGLE_ON_TRUE:
                bindingTrigger.toggleOnTrue(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            case WHILE_FALSE:
                bindingTrigger.whileFalse(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            case WHILE_TRUE:
                bindingTrigger.whileTrue(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            default:
            bindingTrigger.onTrue(new ConditionalBoundCommandContainer(isBindingEnabledSupplier, commandToBind));
                break;
            
        }
    }

    private static class ConditionalBoundCommandContainer extends Command {
        private BooleanSupplier isCommandEnabledSupplier;
        private Command commandToWrap;
        public ConditionalBoundCommandContainer(BooleanSupplier isCommandEnabledSupplier, Command commandToWrap) {
            this.isCommandEnabledSupplier = isCommandEnabledSupplier;
            this.commandToWrap = commandToWrap;
        }

        @Override
        public void initialize() {
            if (isCommandEnabledSupplier.getAsBoolean()) {
                commandToWrap.initialize();
            }
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                commandToWrap.cancel();
            }
        }

        @Override
        public boolean isFinished() {
            if (isCommandEnabledSupplier.getAsBoolean() || commandToWrap.isScheduled()) {
                return commandToWrap.isFinished();
            }
            return true;
        }



    }
}
