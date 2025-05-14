package noncomp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import configs.ServoConfig;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Control with Dashboard Only", group = "Test")
public class IntakeServos extends LinearOpMode {
    private FtcDashboard dashboard;

    private ServoConfig servoConfig;

    public static double intakeRot =0.5;
    public static double clawRot;
    public static double intakeY;
    public static double intakeClaw;

    @Override
    public void runOpMode() {
        servoConfig = new ServoConfig(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {

            servoConfig.intClaw.setPosition(intakeClaw);
            servoConfig.intRot.setPosition(intakeRot);
            servoConfig.intY.setPosition(intakeY);
            servoConfig.intClawRot.setPosition(clawRot);

            telemetry.addData("intakeRot", intakeRot);

            telemetry.update();
        }
    }

    public void setInitPos(){
        if(gamepad1.a){
            servoConfig.setIntakePos(intakeRot, intakeY, clawRot, intakeClaw);
        }
    }
}
