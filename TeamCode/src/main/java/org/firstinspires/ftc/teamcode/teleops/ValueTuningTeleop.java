package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@TeleOp(name = "Launcher RPM Tuning", group = "Tuning")
public class ValueTuningTeleop extends LinearOpMode {

    Launcher launcher;

    double testPower = 0.0;

    final double FINE_STEP = 0.01;
    final double COARSE_STEP = 0.05;

    @Override
    public void runOpMode() {

        DcMotor left = hardwareMap.get(DcMotor.class, "outL");
        DcMotor right = hardwareMap.get(DcMotor.class, "outR");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher = new Launcher(left, right);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) testPower += FINE_STEP;
            if (gamepad1.dpad_down) testPower -= FINE_STEP;
            if (gamepad1.dpad_right) testPower += COARSE_STEP;
            if (gamepad1.dpad_left) testPower -= COARSE_STEP;

            testPower = Math.max(0.0, Math.min(1.0, testPower));

            if (gamepad1.a) {
                launcher.RPMatPowerTest(testPower);
            } else {
                launcher.STOP();
            }

            double rpm = launcher.getRPM();

            telemetry.addData("Power", "%.3f", testPower);
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.addData("Left Encoder", launcher.getLeftEncoder());
            telemetry.addData("Right Encoder", launcher.getRightEncoder());
            telemetry.update();

            sleep(1);
        }

        launcher.STOP();
    }
}
