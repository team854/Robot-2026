import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import java.util.List;
import java.util.ArrayList;

import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.projectile.ProjectileSubsystem;
import frc.robot.subsystems.projectile.ProjectileSubsystem.TargetSolution;

public class ProjectileSubsystemTest {

    private static Stream<Arguments> generateRobotVelocity() {
        List<Arguments> testCases = new ArrayList<>();
        for (double velocity = -10; velocity < 10; velocity++) {
            if (velocity != 0) {
                for (double yaw = 0; yaw < 360; yaw++) {
                    for (double pitch = 10; pitch < 70; pitch++) {
                        testCases.add(Arguments.of(
                            velocity,
                            Degrees.of(yaw),
                            Degrees.of(pitch),
                            (velocity > 0)
                        ));
                    }
                }
            }
        }

        return testCases.stream();
    }

    //@ParameterizedTest
    //@MethodSource("generateRobotVelocity")
    void testRobotVelocity(double velocity, Angle yaw, Angle pitch, boolean moreRange) {
        ProjectileSubsystem testSubsystem = new ProjectileSubsystem();

        double robotVelocityX = velocity * Math.cos(yaw.in(Radian));
        double robotVelocityY = velocity * Math.sin(yaw.in(Radian));

        Translation3d[] baseSim = testSubsystem.simulateLaunch(MetersPerSecond.of(10), pitch, yaw, DegreesPerSecond.of(0), DegreesPerSecond.of(0), new Translation2d(0,0), new Translation3d(0, 0, 0), yaw, 50); 

        Translation3d[] velocitySim = testSubsystem.simulateLaunch(MetersPerSecond.of(10), pitch, yaw, DegreesPerSecond.of(0), DegreesPerSecond.of(0), new Translation2d(robotVelocityX,robotVelocityY), new Translation3d(0, 0, 0), yaw, 50);
        
        // Project crossover point onto target direction vector (dot product)
        double projectedDistanceBase = (baseSim[0].getX() * Math.cos(yaw.in(Radian))) + 
                                  (baseSim[0].getY() * Math.sin(yaw.in(Radian)));

        double projectedDistanceVelocity = (velocitySim[0].getX() * Math.cos(yaw.in(Radian))) + 
                                  (velocitySim[0].getY() * Math.sin(yaw.in(Radian)));

        String failString = "Failed at Pitch: " + pitch.in(Degrees) + " Yaw: " + yaw.in(Degrees) + " Robot Velocity X: " + robotVelocityX + " Robot Velocity Y: " + robotVelocityY;
        if (moreRange) {
            assertTrue(projectedDistanceVelocity > projectedDistanceBase, failString);
        } else {
            assertTrue(projectedDistanceVelocity < projectedDistanceBase, failString);
        }
    }
}