package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class IntakeConstants {

    public static final Slot0Configs swivelGains = new Slot0Configs()
    //57.644
        .withKP(5).withKI(0).withKD(0.0753945)
        .withKS(0.14158).withKV(0.10543).withKA(0.0033477)
        .withGravityType(GravityTypeValue.Arm_Cosine).withKG(0.0036125);

    public static final TalonFXConfiguration swivelConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(15))
                .withStatorCurrentLimitEnable(true)
        );
    
    public static final Slot0Configs intakeGains = new Slot0Configs()
        .withKP(0.0).withKI(0).withKD(0.0)
        .withKS(0.0).withKV(0).withKA(0.0);

    public static final TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(20))
                .withStatorCurrentLimitEnable(true)
        );

    public static final double maxSwivelEncoderTicks = 22.803;
    public static final double minSwivelEncoderTicks = 0;
    public static final double intakeIdle = 15.25;
    public static final double intakeIntake = 20.910;

    public static final double kSwivelGearRatio = 60;
    public static final double kIntakeGearRatio = 9;

    public static final int kSwivelID = 21;
    public static final int kIntakeID = 31;
}
