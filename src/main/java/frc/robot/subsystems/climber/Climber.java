package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final Alert swivelConnected;

    private double swivelGoal;
    private final SysIdRoutine swivelRoutine;
    
    public Climber(ClimberIO io) {        
        this.io = io;
        this.swivelConnected = new Alert("Leader Kraken disconnected for Climber", AlertType.kError);

        this.swivelGoal = 0.0;

        swivelRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, // Default ramp rate is acceptable
                    Volts.of(4),
                    null, // Default timeout is acceptable
                    (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    volts -> io.swivelSetVoltage(volts.in(Volts)), null, this)
                );

        this.io.swivelResetEncoder(0.0);
       // setDefaultCommand(setClimberPositionAndVelocity(0.0, 0.0));
    }

    public Command setPosition(double position) {
        return new FunctionalCommand(
            () -> {
                this.swivelGoal = position;
                Logger.recordOutput("Climber/SwivelGoal", this.swivelGoal);
            },
            () -> io.swivelSetPosition(position),
            interrupted -> {},  
            () -> PhoenixUtil.epsilonEquals(inputs.swivelPosition, position, 0.2),
            this
        );

    }

    public Command setClimberVoltage(double voltage) {
        return runOnce(() -> io.swivelSetVoltage(voltage));
    }

    public Command characterizeSwivel() {
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),

            swivelRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.swivelPosition > 4),

            this.runOnce(() -> io.swivelSetVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            swivelRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.swivelPosition < 1),

            this.runOnce(() -> io.swivelSetVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            swivelRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.swivelPosition > 4),

            this.runOnce(() -> io.swivelSetVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            swivelRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.swivelPosition < 1),

            this.runOnce(() -> SignalLogger.stop())
        );

    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);

        swivelConnected.set(inputs.swivelConnected);

        Logger.processInputs("Climber/Inputs", inputs);
    }
}
