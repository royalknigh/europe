package comp.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import configs.MotorConfig;
import configs.ServoConfig;


@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
@Disabled
public class Merman extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    
    private int fraction;

    @Override
    public void runOpMode() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        fraction =1;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            movement();

        }
    }

    public void movement(){
        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorConfig.setMotorPowers(
                frontLeftPower/fraction,
                backLeftPower/fraction,
                frontRightPower/fraction,
                backRightPower/fraction
        );

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

}
