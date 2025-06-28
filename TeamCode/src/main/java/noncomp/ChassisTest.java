package noncomp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import configs.MotorConfig;

public class ChassisTest extends LinearOpMode {
    MotorConfig motorConfig;

    @Override
    public void runOpMode() throws InterruptedException {
        motorConfig = new MotorConfig(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_up)
                motorConfig.setMotorPowers(1,0,0,0);
            if(gamepad1.dpad_right)
                motorConfig.setMotorPowers(0,0,1,0);
            if(gamepad1.dpad_down)
                motorConfig.setMotorPowers(0,0,0,1);
            if(gamepad1.dpad_left)
                motorConfig.setMotorPowers(0,1,0,0);
        }
    }
}
