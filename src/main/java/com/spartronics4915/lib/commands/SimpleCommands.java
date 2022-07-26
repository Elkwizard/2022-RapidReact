package com.spartronics4915.lib.commands;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.locks.Condition;
import java.util.function.*;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import org.apache.commons.math3.optim.MaxIter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class SimpleCommands {	
	public static class SimpleCommand<T extends CommandBase> {
		protected T mCommand;
		private ArrayList<SpartronicsSubsystem> mRequirements;
		private boolean mRequirementsSet; 

		public SimpleCommand() {
			mRequirementsSet = true;
		}

		public void Requires(SpartronicsSubsystem ...requirements) {
			mRequirementsSet = false;
			mRequirements.addAll(Arrays.asList(requirements));
		}

		public T get() {
			if (!mRequirementsSet) {
				mRequirementsSet = true;
				mCommand.addRequirements((SpartronicsSubsystem[])mRequirements.toArray());
			}
			return mCommand;
		}

		public void run() {
			get().schedule();
		}
	}
	
	public static class ParallelCommand extends SimpleCommand<ParallelCommandGroup> {
		public ParallelCommand() {
			mCommand = new ParallelCommandGroup();
		}

		public ParallelCommand And(SimpleCommand<?> cmd) {
			mCommand.addCommands(cmd.get());
			return this;
		}

		public ParallelCommand And(Runnable action) {
			return And(new BasicCommand(action));
		}
	}

	public static class SerialCommand extends SimpleCommand<SequentialCommandGroup> {
		public SerialCommand() {
			mCommand = new SequentialCommandGroup();
		}
		
		public SerialCommand Then(SimpleCommand<?> cmd) {
			mCommand.addCommands(cmd.get());
			return this;
		}

		public SerialCommand Then(Runnable action) {
			return Then(new BasicCommand(action));
		}
	}

	public static class BasicCommand extends SimpleCommand<CommandBase> {
		private BooleanSupplier mIsFinished;
		private Runnable mAction;

		public BasicCommand(Runnable action) {
			mAction = action;
			mIsFinished = () -> true;
			mCommand = new CommandBase() {
				@Override
				public void execute() {
					mAction.run();
				}

				@Override
				public boolean isFinished() {
					return mIsFinished.getAsBoolean();
				}
			};
		}
		
		public SerialCommand Then(SimpleCommand<?> cmd) {
			return new SerialCommand().Then(this).Then(cmd);
		}

		public SerialCommand Then(Runnable action) {
			return Then(new BasicCommand(action));
		}

		public ParallelCommand And(SimpleCommand<?> cmd) {
			return new ParallelCommand().And(this).And(cmd);
		}

		public ParallelCommand And(Runnable action) {
			return And(new BasicCommand(action));
		}

		public BasicCommand Until(BooleanSupplier condition) {
			mIsFinished = condition;
			return this;
		}
	}
	
	public static class IfCommand extends SimpleCommand<ConditionalCommand> {
		private BooleanSupplier mCondition;
		private SequentialCommandGroup ifTrue;
		private SequentialCommandGroup ifFalse;
		
		public IfCommand(BooleanSupplier condition) {
			mCondition = condition;
			ifTrue = new SequentialCommandGroup();
			ifFalse = new SequentialCommandGroup();
			mCommand = new ConditionalCommand(ifTrue, ifFalse, mCondition);
		}

		public IfCommand Then(SimpleCommand<?> action) {
			ifTrue.addCommands(action.get());
			return this;
		}

		public IfCommand Else(SimpleCommand<?> action) {
			ifFalse.addCommands(action.get());
			return this;
		}
	}

	public static final class Use {
		public static IfCommand If(BooleanSupplier condition) {
			return new IfCommand(condition);
		}

		public static BasicCommand Do(Runnable action) {
			return new BasicCommand(action);
		}
	}
}
