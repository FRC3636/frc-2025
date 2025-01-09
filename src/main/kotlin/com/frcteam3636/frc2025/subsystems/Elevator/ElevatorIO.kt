import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.MotorFFGains
import com.frcteam3636.frc2025.utils.math.PIDGains
import com.frcteam3636.frc2025.utils.math.motorFFGains
import com.frcteam3636.frc2025.utils.math.pidGains
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged

open class ElevatorInputs {
    var height = Meters.zero()!!
    var absoluteEncoderHeight = Meters.zero()!!
    var current = Volts.zero()!!
    var velocity = MetersPerSecond.zero()!!
    var absoluteEncoderConnected = false
}

interface ElevatorIO{

    fun updateInputs(inputs: ElevatorInputs)

    fun runToHeight(height: Distance)

    fun setVoltage(volts: Voltage)

    fun updateHeight(height: Distance)
}

class ElevatorIOReal: ElevatorIO {

    private val elevatorMotor = TalonFX(CTREDeviceId.ElevatorMotor).apply {
        val config = TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Feedback.apply {
                SensorToMechanismRatio = GEAR_RATIO
                FeedbackRotorOffset = 0.0
            }

            Slot0.apply {
                pidGains = PID_GAIN
                motorFFGains = FF_GAINS
                kG = GRAVITY_GAIN
            }

            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY
                MotionMagicAcceleration = PROFILE_ACCELERATION
                MotionMagicJerk = PROFILE_JERK
            }
        }
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        configurator.apply(config)
    }

    private val absoluteEncoder = DutyCycleEncoder(DigitalInput(0))

    override fun updateInputs(inputs: ElevatorInputs) {
        inputs.absoluteEncoderHeight = (METERS_PER_ROTATION * Rotations.of(absoluteEncoder.get())) as Distance
        inputs.absoluteEncoderConnected = absoluteEncoder.isConnected
        inputs.height = (METERS_PER_ROTATION * elevatorMotor.position.value) as Distance
        inputs.velocity = (METERS_PER_ROTATION * elevatorMotor.velocity.value) as LinearVelocity
        inputs.current = elevatorMotor.motorVoltage.value
    }

    override fun runToHeight(height: Distance) {
            Logger.recordOutput("Elevator/Height Setpoint", height)

            var desiredAngle = (height / METERS_PER_ROTATION) as Angle
            var controlRequest = MotionMagicTorqueCurrentFOC(desiredAngle)
            elevatorMotor.setControl(controlRequest)
    }

    override fun setVoltage(volts: Voltage){
        assert(volts in Volts.of(-12.0)..Volts.of(12.0))
        val control = VoltageOut(volts.`in`(Volts))
        elevatorMotor.setControl(control)
    }

    override fun updateHeight(height: Distance) {
        elevatorMotor.setPosition(height.`in`(Meters))
    }

    internal companion object Constants {
        val METERS_PER_ROTATION = Meters.per(Rotation).of(0.0)!!
        private const val GEAR_RATIO = 0.0
        val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
        val FF_GAINS = MotorFFGains(0.0, 0.0, 0.0)
        private const val GRAVITY_GAIN = 0.0
        private const val PROFILE_ACCELERATION = 0.0
        private const val PROFILE_JERK = 0.0
        private const val PROFILE_VELOCITY = 0.0
    }

}