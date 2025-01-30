package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConstants {
    public static final Slot0Configs elevatorGains = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withGravityType(GravityTypeValue.Elevator_Static).withKG(0.0);
    public static final TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );

    public static final double kElevatorGearRatio = 9;
    public static final int kKrakenLeaderPort = 16;
    public static final int kKrakenFollowerPort = 19;
}
