package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.*;

public class ElevatorConstants {
    private static final Slot0Configs elevatorGains = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );

    private static final int kKrakenLeaderPort = 50;
    private static final int kKrakenFollowerPort = 51;
}
