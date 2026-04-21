# Structure
The code follows the standard WPIlib convention of commands and subsystems. Subsystems have been split into folders containing the subsystems and IO files related to the part of the robot.

```
src/main/
├── deploy/
│   
└── java/frc/robot/
    ├── commands/
    │   ├── intake/
    │   │   ├── ActiveIntakeCommand.java
    │   │   ├── DeployIntakeCommand.java
    │   │   ├── RetractIntakeCommand.java
    │   │   └── ToggleIntakeDeployCommand.java
    │   ├── kicker/
    │   │   ├── ActivateKickerCommand.java
    │   │   └── ReverseKickerCommand.java
    │   ├── smart/
    │   │   └── SmartShootCommand.java
    │   ├── spindexer/
    │   │   ├── ActivateSpindexerCommand.java
    │   │   └── ReverseSpindexerCommand.java
    │   └── turret/
    │       ├── ActivateShooterCommand.java
    │       ├── HomeTurretCommand.java
    │       ├── ManualAimCommand.java
    │       ├── ManualStowTurretCommand.java
    │       ├── ResetTurretPitchCommand.java
    │       ├── ToggleFixedAimCommand.java
    │       ├── ToggleManualAimCommand.java
    │       └── TurretAutoAimCommand.java
    ├── libraries/
    │   ├── control/
    │   │   ├── ControllerIO.java
    │   │   ├── ControllerIOPS5.java
    │   │   └── ControllerIOXbox.java
    │   ├── FieldHelpers.java
    │   ├── LimelightHelpers.java
    │   ├── PoseHelpers.java
    │   ├── ProjectileSimulation.java
    │   ├── StateMachine.java
    │   └── SubsystemStateMachine.java
    ├── subsystems/
    │   ├── drive/
    │   │   ├── SwerveIO.java
    │   │   ├── SwerveIOReal.java
    │   │   └── SwerveSubsystem.java
    │   ├── intake/
    │   │   ├── IntakeDeploymentIO.java
    │   │   ├── IntakeDeploymentIOReal.java
    │   │   ├── IntakeDeploymentSubsystem.java
    │   │   ├── IntakeIO.java
    │   │   ├── IntakeIOReal.java
    │   │   └── IntakeSubsystem.java
    │   ├── lights/
    │   │   └── LightSubsystem.java
    │   ├── logging/
    │   │   ├── HealthSubsystem.java
    │   │   └── VisualizerSubsystem.java
    │   ├── spindexer/
    │   │   ├── SpindexerIO.java
    │   │   ├── SpindexerIOReal.java
    │   │   └── SpindexerSubsystem.java
    │   ├── turret/
    │   │   ├── CalculationSubsystem.java
    │   │   ├── KickerIO.java
    │   │   ├── KickerIOReal.java
    │   │   ├── KickerSubsystem.java
    │   │   ├── ShooterIO.java
    │   │   ├── ShooterIOKrakenReal.java
    │   │   ├── ShooterIOReal.java
    │   │   ├── ShooterSubsystem.java
    │   │   ├── TurretIO.java
    │   │   ├── TurretIOKrakenReal.java
    │   │   ├── TurretIOReal.java
    │   │   └── TurretSubsystem.java
    │   └── vision/
    │       ├── LimelightSubsystem.java
    │       └── QuestNavSubsystem.java
    ├── Constants.java
    ├── ErrorConstants.java
    ├── Main.java
    ├── Robot.java
    └── RobotContainer.java
```

# IO abstraction layer

Each subsystem that controls hardware with the exception of `LimelightSubsystem.java`, `QuestNavSubsystem.java`, and `LightSubsystem.java` uses an IO abstraction layer. This has several advantages for the code:
 - It makes the subsystem code cleaner
 - It allows subsystems to be disabled by passing in the `____IO.java` instead of an `____IOReal.java`
 - It allows for the hardware to be easily switched for example `ShooterIOReal`(NEOs) and `ShooterIOKrakenReal.java`(Krakens)
 - For our team specifically it allows testing code on incomplete hardware

Each IO abstraction layer consists of a base `____IO.java` class and a class with the real implementation `____IOReal.java`. For example the spindexer uses `SpindexerIO.java` and `SpindexerIOReal.java`

### SpindexerIO.java:
```java
public interface SpindexerIO {
    default void setMotorVoltage(double voltage) {}

    default double getMotorCurrent() {return 0;}

    default boolean checkCANError() {return false;}
}
```

### SpindexerIOReal.java:
```java
public class SpindexerIOReal implements SpindexerIO {
    private final SparkMaxConfig spindexerConfig;
    private final SparkMax spindexerMotor;

    public SpindexerIOReal() {
        spindexerMotor = new SparkMax(Constants.SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

        spindexerConfig = new SparkMaxConfig();
        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.inverted(Constants.SpindexerConstants.SPINDEXER_MOTOR_INVERTED);
        spindexerConfig.smartCurrentLimit(40); 
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        spindexerMotor.setVoltage(voltage);
    }

    @Override
    public double getMotorCurrent() {
        return spindexerMotor.getOutputCurrent();
    }

    @Override
    public boolean checkCANError() {
        spindexerMotor.getBusVoltage();
        if (spindexerMotor.getFaults().can == true) {
            return true;
        }

        return false;
    }
}
```

The `SpindexerIO.java` class is an interface that implements default functions. Default getter functions return safe values that won't crash the subsystem or cause adverse effects. Setter functions accept all of the arguments of the normal setter then discard the values and do nothing. In `SpindexerIOReal.java` each of the default methods is implemented via a `@Override`

# Statemachines
Each of the subsystems that controls hardware with the exception of `SwerveSubsystem.java`, `LimelightSubsystem.java`, `QuestNavSubsystem.java`, and `LightSubsystem.java` extends a custom statemachine called `SubsystemStateMachine` (Code can be found in `/src/main/java/frc/robot/libraries/SubsystemStateMachine.java`) which itself extends `SubsystemBase` and provides a proxy for the custom `StateMachine` (Code can be found in `/src/main/java/frc/robot/libraries/StateMachine.java`). State machines have several advantages for the code:
 - It centralizes the state switching logic
 - It adds a priority system so multiple systems requesting a state get automatically prioritized
 - It reduces conditions where the robot goes into weird states that are hard to debug

The state machine operates on a `desiredState` and a `currentState`. The `desiredState` is the state that the subsystem wants to be in and the `currentState` is the state that the subsystem is in. When something requests a desired state it passes in both the `desiredState` and a `priority`. At the start of each periodic tick the state machine sets the current `desiredState` to the requested `desiredState` with the highest priority. In this code I have structured the priorities as follows:
 - 0: Idle state
 - 1-10: Normal commands
 - 11-20: Manual overrides
 - 21-30: Safety overrides

While the architecture is pretty elegant and worked very well there are some important things to remember:
 1. It takes one tick to switch `desiredState`. Meaning if a state is requested on tick 0 it will only be set as the `desiredState` on tick 1
 2. In the state machines default configuration it will automatically reset its `desiredState` to the `defaultState` if no requests for a `desiredState` are made in a tick. This behaviour can be disabled by passing in `null` to the `defaultState` upon constructing the state machine
 3. Only the subsystem that extends the `SubsystemStateMachine` can transition states using the `transitionTo()`. If you find yourself in a state where this needs to be violated you should probably use the base `StateMachine` class

### SpindexerSubsystem.java:
```java
public class SpindexerSubsystem extends SubsystemStateMachine<frc.robot.subsystems.spindexer.SpindexerSubsystem.SpindexerState> {

    public enum SpindexerState {
        IDLE,
        STOWED,
        READY_REVERSE,
        READY,
    }

    private final SpindexerIO io;

    private double lastErrorTimestamp = Double.NEGATIVE_INFINITY;

    public SpindexerSubsystem(SpindexerIO io) {
        super(SpindexerState.IDLE, SpindexerState.IDLE);

        if (io == null) {
            throw new IllegalArgumentException("SpindexerIO cannot be null");
        }

        this.io = io;
    }

    public Current getMotorCurrent() {
        return Amp.of(io.getMotorCurrent());
    }

    public void checkCanHealth() {
        double timestamp = Timer.getFPGATimestamp();
        if (io.checkCANError()) {
            lastErrorTimestamp = timestamp;
        }

        if ((timestamp - lastErrorTimestamp) < Constants.HealthConstants.CAN_ERROR_PERSIST.in(Second)) {
            RobotContainer.healthSubsystem.reportError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        } else {
            RobotContainer.healthSubsystem.clearError(getSubsystem(), ErrorConstants.MOTOR_CAN_ERROR);
        }
    }

    @Override
    public void statePeriodicBefore() {
        if (RobotContainer.calculationSubsystem.getZone() == Zone.TRENCH) {
            requestDesiredState(SpindexerState.STOWED, 30);
        } else {
            requestDesiredState(SpindexerState.IDLE, 0);
        }
    }

    @Override
    public void statePeriodic() {

        switch (getCurrentState()) {
            case IDLE:
                if (getDesiredState() == SpindexerState.STOWED) {
                    transitionTo(SpindexerState.STOWED);
                } else if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                }
                break;
            case STOWED:
                if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                } else if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }
                break;

            case READY_REVERSE:
                if (getDesiredState() == SpindexerState.STOWED) {
                    transitionTo(SpindexerState.STOWED);
                } else if (getDesiredState() == SpindexerState.READY) {
                    transitionTo(SpindexerState.READY);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }

                break;
            case READY:
                if (getDesiredState() == SpindexerState.STOWED) {
                    transitionTo(SpindexerState.STOWED);
                } else if (getDesiredState() == SpindexerState.READY_REVERSE) {
                    transitionTo(SpindexerState.READY_REVERSE);
                } else if (getDesiredState() == SpindexerState.IDLE) {
                    transitionTo(SpindexerState.IDLE);
                }

                break;
        }

        double spindexerVoltage = 0.0;
        switch (getCurrentState()) {
            case IDLE:
                spindexerVoltage = 0;
                break;
            case STOWED:
                spindexerVoltage = 0;
                break;
            case READY_REVERSE:
                spindexerVoltage = -Constants.SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE.in(Volt);
                break;
            case READY:
                spindexerVoltage = Constants.SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE.in(Volt);
                break;
            default:
                spindexerVoltage = 0.0;
                System.err.println("Spindexer in unknown state: " + getCurrentState());
                break;
        }

        spindexerVoltage = MathUtil.clamp(spindexerVoltage, -10, 10);
        io.setMotorVoltage(spindexerVoltage);

        checkCanHealth();

        SmartDashboard.putNumber("Spindexer/Voltage", spindexerVoltage);

        SmartDashboard.putNumber("Spindexer/Current", io.getMotorCurrent());

        SmartDashboard.putString("Spindexer/Current State", getCurrentState().name());
        SmartDashboard.putString("Spindexer/Desired State", getDesiredState().name());
    }
}
```

The class extends `SubsystemStateMachine` and passes a type of an `Enum` containing all the posible states of the subsystem. In the constructor of `SpindexerSubsystem.java` it calls the super of the `SubsystemStateMachine` and passes in the `startingState` and the `defaultState`. The `startingState` controls what state the state machine starts in and the `defaultState` controls what state the `desiredState` resets to if no `desiredState` is requested in a tick. The code calls `statePeriodicBefore` then the `statePeriodic`. `statePeriodicBefore` is called before the previous ticks `desiredState` is applied and `statePeriodic` is called after. In `statePeriodic` the subsystem evalulates if it should transition from each state to each other state. Then it executes the code for each current state.

# Units
WPILib provides a helpful `Units` library that allows unit safe varibles. This provides several advantages for the code:
 - Prevent mistakes involving incorect units in calculations
 - Automatically converts between units
 - Makes the codes intent much more explicit
 - Prevents passing in the wrong units into a function

The `Units` library provides a set of predefined unit types which includes `Distance`, `Angle`, `LinearVelocity`, `Time`, and `Mass`. These units are further split into specific units. For example to set a distance you would do:
```java
Distance exampleDistance = Meters.of(1.423);
```
Then to read that distance back into a double you would do:
```java
double exampleDistanceMeter = exampleDistance.in(Meters);
double exampleDistanceInches = exampleDistance.in(Inch);
```
I would strongly recomend you use the `Units` library in your code. You can find documentation [here](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html). Do note that while in some cases it is convient to use imperial units in `Constants.java` all WPIlib functions use metric so so I would advise using metric internally. Its also important to note that the `Units` library does cause garbage collection pressure when used in frequent loops.

# Constants
This project uses a central file called `Constants.java` to store the constants for every part of the code. Just like how the subsystem files are organized into folders, the constants are organized into sub classes.
```java
public final class Constants {
    public final class SpindexerConstants {
        public static final boolean ENABLED = true;
        public static final int SPINDEXER_MOTOR_ID = 29;
        public static final boolean SPINDEXER_MOTOR_INVERTED = true;
        public static final Voltage SPINDEXER_MOTOR_VOLTAGE = Volt.of(12);
        public static final Current SPINDEXER_MOTOR_CURRENT_LIMIT = Amp.of(30);
    }
}
```

# Localization
For the robot to perform well in a competition it needs to know precisly where it is on the field. This robot uses Wheel Odometry, Limelight, and QuestNav.
## Wheel Odometry
Wheel odometry updates the fastest and is what the robot will use most of the time. Wheel odometry is accurate but it has a few caveats namly its inability to deal with wheel slip, it only works when on the ground, and its dependance on knowing its starting position. Wheel odometry is what is known as relative positioning. It knows where it is relative to its starting position but not its actual position.
## Limelight
Limelights are used to give the robot absolute position on the field. It updates slower than wheel odometry but its main advantage is that it provides absolute position on the field. This does come with the caveat that the limelight needs to see an april tag to know where it is on the field. The number of tags that the limelight can see will influence how accurate its position is. With one tag, even when using `MegaTag2` the limelights position is not that accurate. With two or more tags the limelights position becomes much more accurate. One of the most important parts of limelight localization is determining the standard deviations of the limelights position. Standard deviations represent how confident the robot should be in the limelights mesuremen with lower values representing higher confidence. Here is how it is implemented in this code
```java
public VisionStdDevs calculateVisionStdDevs(Pose2d visionPose, int tagCount, double averageTagDistance, ChassisSpeeds robotChassisSpeeds, AngularVelocity robotAngularVelocity) {
    if (tagCount == 0) {
        return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.NO_TAGS);
    }

    if (Math.abs(robotAngularVelocity.in(RadiansPerSecond)) > (Math.PI * 2)) {
        return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.MAX_ANGULAR_VELOCITY);
    }

    if (!FieldHelpers.poseInField(visionPose)) {
        return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.OUT_OF_FIELD_BOUNDS);
    }
    
    double robotTranslationalVelocity = Math.hypot(robotChassisSpeeds.vxMetersPerSecond, robotChassisSpeeds.vyMetersPerSecond);

    double stdDevs = Math.abs(robotAngularVelocity.in(DegreesPerSecond)) / Constants.LimelightConstants.ANGULAR_VELOCITY_DIVISOR;

    stdDevs += robotTranslationalVelocity / Constants.LimelightConstants.TRANSLATIONAL_VELOCITY_DIVISOR;

    if (tagCount == 1) {
        if (averageTagDistance >= 5) {
            return new VisionStdDevs(Double.MAX_VALUE, VisionRejection.SINGLE_TAG_MAX_DISTANCE);
        }

        stdDevs += (Constants.LimelightConstants.SINGLE_TAG_STARTING_STD_DEV + (Math.pow(averageTagDistance, 2.0) * Constants.LimelightConstants.SINGLE_TAG_DISTANCE_FACTOR));
    } else {
        stdDevs += (Constants.LimelightConstants.MULTI_TAG_STARTING_STD_DEV + (averageTagDistance * Constants.LimelightConstants.MULTI_TAG_DISTANCE_FACTOR));
    }

    stdDevs = Math.max(stdDevs, 0.05);

    return new VisionStdDevs(stdDevs, VisionRejection.NONE);
}
```
This code calculates the std devs of a limelight mesurement based on `visionPose`, `tagCount`, `averageTagDistance`, `robotChassisSpeeds`, and `robotAngularVelocity`. The code first filters for obvious bad mesurements like when the mesurement is out of the field perimeter. Then it penalizes the mesurement based on how fast the robot is translating and rotating. The final and most important step is penalizing the mesurement based on the average distance of the tags. If there is only 1 tag then the mesurement should be penalized much more.

# Logging
While there are many ways to handle logging in frc this code uses `SmartDashboard` to publish values over `NetworkTables`. In addition it uses `AdvantageKit` to log additional values and provide better replay compatiblity. `AdvantageKit` is able to log completly without `SmartDashboard` but I chose to use `SmartDashboard` because I was familar with it. I would encourge you to do some reserch and decide on your own setup. Some important things to note with `AdvantageKit` is that you should be logging `NetworkTables` as part of your log file. For example:
```java
public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer;
	
	public Robot() {
		robotContainer = new RobotContainer();

		Logger.recordMetadata("ProjectName", "2026 Robot");

		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter());
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			setUseTiming(false);
		}
		Logger.start(); 
	}

    @Override
    public void testInit() {
        robotContainer.initAll();
    }
    @Override
    public void teleopInit() {
        robotContainer.initAll();
    }
    @Override
    public void autonomousInit() {
        robotContainer.initAll();
    }
}
```
```java
public class RobotContainer {
    public void initAll() {
		if (Robot.isReal()) {
			DataLogManager.start();
			DataLogManager.logNetworkTables(true); 
		}
    }
}
```
# Controller IO Abstraction
Just like how subsystems use IO abstractions for their hardware, abstracting the controllers can have several advantages for the code:
 - Allows interchangeable use of both PS5 and XBox controllers
 - Allows applying a curve to both controller types easily

In this code `ControllerIO.java` is the base class that contains default methods that return safe values while `ControllerIOPS5.java` and `ControllerIOXbox.java` implment the required methods from `ControllerIO.java`. In `RobotContainer.java` switching bettween PS5 and Xbox is simple:
```java
public class RobotContainer {
	public static final ControllerIO driverController = Robot.isReal() ? new ControllerIOPS5(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) : new ControllerIOPS5(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
}
```

# Projectile Simulation

To calculate the required launch pitch, yaw, and speed of the projectile, this code uses a RK4(Runge-Kutta 4th Order) physics simulation and Broyden's method. The physics simulation accounts for translational drag, rotational drag, the magnus effect, and shooter offsets. This has several advantages for the code:
 - Built in SOTM(Shoot On The Move)
 - Easy to tune for a new or incomplete robot
The projectile simulation uses RK4 which allows it to run at incredibly low tps while still remaining stable. The projectile simulation code can be found in `src/main/java/frc/robot/libraries/ProjectileSimulation.java` and the simulation itself is handled by the `simulateLaunch` method. One of the most difficult parts of the simulation was optimizing it for the robot rio. The robo rio is incredably underpowered as its CPU has 2 cores that run at 667 MHz. To optimize the code I had to use several strategies:
 - All varibles in the main simulation loop are declared once then over written.
 - No dynamic length arrays
 - All varibles are primatives
 - No use of the `Units` library in the hot path

To optimize the pitch and yaw the code first calculates the pitch and yaw asuming no drag, robot movement, or magnus effect. 