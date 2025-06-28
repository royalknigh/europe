package noncomp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.List;

import configs.ServoConfig;
import consts.IntConst;

@TeleOp(name = "Sample Orientation Detector", group = "Limelight")
public class SampleDetectionTest extends LinearOpMode {

    private Limelight3A limelight;
    private ServoConfig servoConfig;

    @Override
    public void runOpMode() {
        servoConfig = new ServoConfig(hardwareMap);
        // Make sure "limelight" and "rotationServo" match your robot configuration names
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(4);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            servoConfig.intY.setPosition(0.5);
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                List<List<Double>> corners = detection.getTargetCorners();

                if (corners.size() == 4) {
                    // Each corner is a List<Double> with [x, y]
                    double x0 = corners.get(0).get(0);
                    double y0 = corners.get(0).get(1);
                    double x1 = corners.get(1).get(0);
                    double y1 = corners.get(1).get(1);
                    double x2 = corners.get(2).get(0);
                    double y2 = corners.get(2).get(1);
                    double x3 = corners.get(3).get(0);
                    double y3 = corners.get(3).get(1);

                    // Calculate width as distance between corner 0 and corner 1
                    double width = Math.hypot(x1 - x0, y1 - y0);
                    // Calculate height as distance between corner 1 and corner 2
                    double height = Math.hypot(x2 - x1, y2 - y1);

                    telemetry.addData("Width", width);
                    telemetry.addData("Height", height);

                    if (width - height > 80) {
                        telemetry.addLine("Orientation: Horizontal");
                        servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                    } else {
                        telemetry.addLine("Orientation: Vertical");
                        servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);

                    }
                }


            telemetry.update();
        }
    }
    }

}