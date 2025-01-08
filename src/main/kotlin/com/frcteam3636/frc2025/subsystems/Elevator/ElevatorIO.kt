import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.TalonFX
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
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

class ElevatorIOReal: ElevatorIO{

    private val elevatorMotor = TalonFX(CTREDeviceId.LeftArmMotor)

    private val absoluteEncoder = DutyCycleEncoder(DigitalInput(0))

    override fun updateInputs(inputs: ElevatorInputs) {
        inputs.absoluteEncoderHeight = Meters.of(absoluteEncoder.absolutePosition)
        inputs.absoluteEncoderConnected = absoluteEncoder.isConnected
        inputs.height = (METERS_PER_ROTATION * elevatorMotor.position.value) as Distance
    }

    override fun runToHeight(height: Distance) {

    }

    override fun setVoltage(volts: Voltage){

    }

    override fun updateHeight(height: Distance) {

    }

    init {

    }

    internal companion object Constants {
        val METERS_PER_ROTATION = Meters.per(Rotation).of(0.0)!!
    }

}