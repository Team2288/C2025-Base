package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class IntakeConstants {

    public static final Slot0Configs swivelGains = new Slot0Configs()
        .withKP(0.0).withKI(0).withKD(0.0)
        .withKS(0.0).withKV(0).withKA(0.0)
        .withGravityType(GravityTypeValue.Arm_Cosine).withKG(0.0);

    public static final TalonFXConfiguration swivelConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    
    public static final Slot0Configs intakeGains = new Slot0Configs()
        .withKP(0.0).withKI(0).withKD(0.0)
        .withKS(0.0).withKV(0).withKA(0.0);

    public static final TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
        );

    public static final double maxSwivelEncoderTicks = 0;
    public static final double minSwivelEncoderTicks = 0;
    public static final double intakeIdle = 0.0;

    public static final double kSwivelGearRatio = 0;
    public static final double kIntakeGearRatio = 0;

    public static final int kSwivelID = 0;
    public static final int kIntakeID = 0;
}
