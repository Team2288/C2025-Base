package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class ClimberConstants {

    public static final Slot0Configs swivelGains = new Slot0Configs()
    //57.644
        .withKP(215).withKI(100).withKD(0.0)
        .withKS(0.0).withKV(0.0).withKA(0.0)
        .withGravityType(GravityTypeValue.Arm_Cosine).withKG(13);

    public static final TalonFXConfiguration swivelConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true)
        );
    
    public static final double maxSwivelEncoderTicks = 5.4975;
    public static final double minSwivelEncoderTicks = 0;
    public static final double swivelClimb = 5.6;
    public static final double swivelIdle = -0.0;

    public static final double kSwivelGearRatio = 100;

    public static final int kSwivelID = 18;
}
