package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConstants {
    public static final Slot0Configs elevatorGains = new Slot0Configs()
        .withKP(13).withKI(0).withKD(0.14896)
        .withKS(0.15199).withKV(1.097).withKA(0.017009).withGravityType(GravityTypeValue.Elevator_Static).withKG(0.017);

    public static final TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );

    public static final double maxEncoderTicks = 4.6;
    public static final double minEncoderTicks = 0;

    public static final double kElevatorGearRatio = 9;
    public static final int kKrakenLeaderPort = 19;
    public static final int kKrakenFollowerPort = 16;
}