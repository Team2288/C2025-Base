
package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
    TalonFX krakenIntake, krakenSwivel;

    // intake requests
    private final VoltageOut intakeVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage intakeMotionVoltageRequest = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);

    // swivel requests
    private final VoltageOut swivelVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVoltage swivelMotionVoltageRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

    // Inputs from intake motor
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeAppliedVolts;
    private final StatusSignal<Current> intakeCurrent;

    // Inputs from swivel motor
    private final StatusSignal<Angle> swivelPosition;
    private final StatusSignal<AngularVelocity> swivelVelocity;
    private final StatusSignal<Voltage> swivelAppliedVolts;
    private final StatusSignal<Current> swivelCurrent;

    // Connection debouncers
    private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);
    private final Debouncer swivelConnectedDebounce = new Debouncer(0.5);

    public IntakeIOTalonFX() {
        krakenIntake = new TalonFX(IntakeConstants.kIntakeID);
        krakenSwivel = new TalonFX(IntakeConstants.kSwivelID);

        var intakeConfig = IntakeConstants.intakeConfiguration;

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.Slot0 = IntakeConstants.intakeGains;
        intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kIntakeGearRatio;

        intakeConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        intakeConfig.MotionMagic.MotionMagicAcceleration = 400;
        intakeConfig.MotionMagic.MotionMagicJerk = 0;

        var swivelConfig = IntakeConstants.swivelConfiguration;

        swivelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swivelConfig.Slot0 = IntakeConstants.swivelGains;
        intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kSwivelGearRatio;

        swivelConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        swivelConfig.MotionMagic.MotionMagicAcceleration = 400;
        swivelConfig.MotionMagic.MotionMagicJerk = 0;

        tryUntilOk(5, () -> krakenIntake.getConfigurator().apply(intakeConfig, 0.25));
        tryUntilOk(5, () -> krakenIntake.setPosition(0.0, 0.25));

        tryUntilOk(5, () -> krakenSwivel.getConfigurator().apply(swivelConfig, 0.25));
        tryUntilOk(5, () -> krakenSwivel.setPosition(0.0, 0.25));

        // Create intake status signals
        intakeVelocity = krakenIntake.getVelocity();
        intakeAppliedVolts = krakenIntake.getMotorVoltage();
        intakeCurrent = krakenIntake.getStatorCurrent();

        // Create swivel status signals
        swivelPosition = krakenSwivel.getPosition();
        swivelVelocity = krakenSwivel.getVelocity();
        swivelAppliedVolts = krakenSwivel.getMotorVoltage();
        swivelCurrent = krakenSwivel.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            intakeVelocity,
            intakeAppliedVolts,
            intakeCurrent,
            swivelPosition,
            swivelVelocity,
            swivelAppliedVolts,
            swivelCurrent);
        ParentDevice.optimizeBusUtilizationForAll(krakenIntake, krakenSwivel);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Refresh all signals
        var intakeStatus =
            BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrent);
        var swivelStatus =
            BaseStatusSignal.refreshAll(swivelPosition, swivelVelocity, swivelAppliedVolts, swivelCurrent);

        // Update intake inputs
        inputs.intakeConnected = intakeConnectedDebounce.calculate(intakeStatus.isOK());
        inputs.intakeVelocityRotPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
        inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();

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

    @Override
    public void intakeSetVoltage(double volts) {
        krakenIntake.setControl(intakeVoltageRequest.withOutput(volts));
    }

    @Override
    public void intakeSetVelocity(double velocityRotPerSec) {
        krakenIntake.setControl(intakeMotionVoltageRequest.withVelocity(velocityRotPerSec));
    }
}