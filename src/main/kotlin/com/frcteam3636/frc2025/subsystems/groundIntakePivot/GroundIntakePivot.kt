object GroundIntakePivot : Subsystem {
    private val io: GroundIntakePiviotIO = when (Robot.model) {
        Robot.Model.SIMULATION -> PivotlIOSim()
        Robot.Model.COMPETITION -> PivotIOReal()
        Robot.Model.PROTOTYPE -> TODO()
    }

    var inputs = LoggedGroundIntakePivotInputs()

    override fun  periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Pivot", inputs)
    }

    fun setTargetHeight(position: Position): Command =
    start
}