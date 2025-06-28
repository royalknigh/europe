package Components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorComponent {
    private Rev2mDistanceSensor sensor;
    private double cachedDistance;

    public DistanceSensorComponent(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, name);
    }

    public void poll() {
        cachedDistance = sensor.getDistance(DistanceUnit.CM);
    }

    public double getDistance() {
        return cachedDistance;
    }
}
