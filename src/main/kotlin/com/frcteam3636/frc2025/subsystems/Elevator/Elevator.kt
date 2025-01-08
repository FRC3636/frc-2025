import com.frcteam3636.frc2025.Robot
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

private const val SECONDS_BETWEEN_ELEVATOR_UPDATES = 0.5

object Elevator: Subsystem {
    private val io: ElevatorIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ElevatorIOSim()
        Robot.Model.COMPETITION -> ElevatorIOReal()
        Robot.Model.PROTOTYPE -> ElevatorIOReal()
    }

    var inputs = LoggedElevatorInputs()

    private var timer = Timer().apply {
        start()
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)
        if (timer.advanceIfElapsed(SECONDS_BETWEEN_ELEVATOR_UPDATES) && inputs.absoluteEncoderConnected){
            io.updateHeight(inputs.absoluteEncoderHeight)
        }
    }

    fun setTargetHeight(position: Position): Command =
        startEnd({
        io.runToHeight(position.height)
    }, {
        io.runToHeight(inputs.height)
    })!!

//    fun sysIdQuasistatic(direction: Direction) =
//        sysID.quasistatic(direction)!!
//
//    fun sysIdDynamic(direction: Direction) =
//        sysID.dynamic(direction)!!

    enum class Position(val height: Distance) {
        Stowed(Meters.of(0.0)),
        Trough(Meters.of(0.0)),
        LowBar(Meters.of(0.0)),
        MidBar(Meters.of(0.0)),
        HighBar(Meters.of(0.0)),
        LowAlgae(Meters.of(0.0)),
        HighAlgae(Meters.of(0.0)),
    }
}