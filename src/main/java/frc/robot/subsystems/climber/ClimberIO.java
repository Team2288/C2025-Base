package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean swivelConnected = false;

        public double swivelPosition = 0.0;
        public double swivelVelocityRadPerSec = 0.0;
        public double swivelAppliedVolts = 0.0;
        public double swivelCurrentAmps = 0.0;
    }   

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void swivelSetPosition(double position) {}
    public default void swivelSetVoltage(double voltage) {}
    public default void swivelResetEncoder(double ticks) {}

    public default void ClimberSetVelocity(double velocity) {}
    public default void ClimberSetVoltage(double voltage) {}
}
