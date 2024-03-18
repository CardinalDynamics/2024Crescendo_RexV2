package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rotator;

public class DefaultRotatorCommand extends Command {
    Rotator m_rotator;
    BooleanSupplier isManual;
    DoubleSupplier rotatorSpeed;
    double usedSpeed;

    DefaultRotatorCommand(Rotator rotator, BooleanSupplier mode, DoubleSupplier speed) {
        m_rotator = rotator;
        isManual = mode;
        rotatorSpeed = speed;
        addRequirements(m_rotator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (rotatorSpeed.getAsDouble() < .05 && rotatorSpeed.getAsDouble() > -.05) {
            usedSpeed = 0;
        } else {
            usedSpeed = rotatorSpeed.getAsDouble() / 10;
        }
        if (isManual.getAsBoolean()) {

        }
    }
}