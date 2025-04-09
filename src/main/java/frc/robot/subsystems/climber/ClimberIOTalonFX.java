
package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumber;

public class ClimberIOTalonFX implements ClimberIO {
    TalonFX krakenSwivel;
    private static final TunableNumber kP = new TunableNumber("Climber/kP");
    private static final TunableNumber kD = new TunableNumber("Climber/kD");
    private static final TunableNumber kG = new TunableNumber("Climber/kG");
    private static final TunableNumber kI = new TunableNumber("Climber/kI");

    private static final TunableNumber maxVel = new TunableNumber("Climber/maxVel");
    private static final TunableNumber maxAccel = new TunableNumber("Climber/maxAccel");


    // swivel requests
    private final VoltageOut swivelVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVoltage swivelMotionVoltageRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

    // Inputs from swivel motor
    private final StatusSignal<Angle> swivelPosition;
    private final StatusSignal<AngularVelocity> swivelVelocity;
    private final StatusSignal<Voltage> swivelAppliedVolts;
    private final StatusSignal<Current> swivelCurrent;
    private TalonFXConfiguration swivelConfig;

    // Connection debouncers
    private final Debouncer swivelConnectedDebounce = new Debouncer(0.5);

    static {
        kP.setDefault(215);
        kD.setDefault(0.0);
        kG.setDefault(13);
        kI.setDefault(100);
        maxVel.setDefault(800);
        maxAccel.setDefault(2000);

    }

    public ClimberIOTalonFX() {
        
        krakenSwivel = new TalonFX(ClimberConstants.kSwivelID);

        swivelConfig = ClimberConstants.swivelConfiguration;

        swivelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swivelConfig.Slot0 = ClimberConstants.swivelGains;
        swivelConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kSwivelGearRatio;

        swivelConfig.MotionMagic.MotionMagicCruiseVelocity = 200 * 144/60;
        swivelConfig.MotionMagic.MotionMagicAcceleration = 600 * 144/60;
        swivelConfig.MotionMagic.MotionMagicJerk = 0;

        tryUntilOk(5, () -> krakenSwivel.getConfigurator().apply(swivelConfig, 0.25));

        // Create swivel status signals
        swivelPosition = krakenSwivel.getPosition();
        swivelVelocity = krakenSwivel.getVelocity();
        swivelAppliedVolts = krakenSwivel.getMotorVoltage();
        swivelCurrent = krakenSwivel.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            swivelPosition,
            swivelVelocity,
            swivelAppliedVolts,
            swivelCurrent);
        ParentDevice.optimizeBusUtilizationForAll(krakenSwivel);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Refresh all signals
        var swivelStatus =
            BaseStatusSignal.refreshAll(swivelPosition, swivelVelocity, swivelAppliedVolts, swivelCurrent);

        if (swivelConfig.Slot0.kP != kP.get() || swivelConfig.Slot0.kD != kD.get() || swivelConfig.Slot0.kG != kG.get() || swivelConfig.MotionMagic.MotionMagicAcceleration != maxAccel.get() || swivelConfig.MotionMagic.MotionMagicCruiseVelocity != maxVel.get() || swivelConfig.Slot0.kI != kI.get()) {
            swivelConfig.Slot0.kP = kP.get();
            swivelConfig.Slot0.kD = kD.get();
            swivelConfig.Slot0.kG = kG.get();
            swivelConfig.Slot0.kI = kI.get();
            swivelConfig.MotionMagic.MotionMagicAcceleration = maxAccel.get();
            swivelConfig.MotionMagic.MotionMagicCruiseVelocity = maxVel.get();
    
            tryUntilOk(5, () -> krakenSwivel.getConfigurator().apply(swivelConfig, 0.25));
    
        }
        
        // Update swivel inputs
        inputs.swivelConnected = swivelConnectedDebounce.calculate(swivelStatus.isOK());
        inputs.swivelPosition = swivelPosition.getValueAsDouble();
        inputs.swivelVelocityRadPerSec = Units.rotationsToRadians(swivelVelocity.getValueAsDouble());
        inputs.swivelAppliedVolts = swivelAppliedVolts.getValueAsDouble();
        inputs.swivelCurrentAmps = swivelCurrent.getValueAsDouble();
    }

    @Override
    public void swivelSetPosition(double position) {
        krakenSwivel.setControl(swivelMotionVoltageRequest.withPosition(position));
    }

    @Override
    public void swivelResetEncoder(double position) {
        krakenSwivel.setPosition(position);
    }

    @Override
    public void swivelSetVoltage(double volts) {
        krakenSwivel.setControl(swivelVoltageRequest.withOutput(volts));
    }

}