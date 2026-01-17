package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.abs;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.utilscripts.CamHandler;
import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name="OLD - Decode Blue", group="99 - Depricated scripts")
public class OldBlueTeleop extends LinearOpMode {
    final boolean DEBUG = true; //true;

    private final double encoderPPR = 537.7 * 2; // the gear ratio

    private VoltageSensor expansionHubVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");

        double target = 0;
        PIDController manipPID = new PIDController(0.0075, 0, 0); //0.005); // TODO - Tune these!

        DcMotor manipulator = hardwareMap.dcMotor.get("bigWheel");
        manipulator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manipulator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manipulator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manipulator.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean lastBumpL = false;
        boolean lastBumpR = false;
        boolean lastShift = false;


        // Reverse the right side motors.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //STOP THE DRIFT
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        float nominalVoltage = 13;
        double CurrentVoltagePercentage = 0;

        expansionHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 1");

        DcMotor leftLauncher = hardwareMap.dcMotor.get("outL");
        DcMotor rightLauncher = hardwareMap.dcMotor.get("outR");

        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE); // make both spin same way

        manipPID.setTarget(manipulator.getCurrentPosition()); // set the wheels init position to the target so it docent kill anybody

        //CamHandler camera = new CamHandler(hardwareMap.get(HuskyLens.class,"camera"), true);//TODO - meue for choising red or blude

        //CamHandler camera = new CamHandler(hardwareMap.get(HuskyLens.class, "camera"), true, telemetry);

        double gameOffset = 0;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, I have no idea why every stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean opt = gamepad1.options;


            boolean camAim = false;
            boolean imuAim = false;
            double launchStrength = 0;

            if(gamepad2.dpad_right) {
                launchStrength = 0.35;// big half
                camAim = true;
            }else if (gamepad2.dpad_down) {
                launchStrength = 0.35; // wall
            } else if (gamepad2.dpad_up) {
                launchStrength = 0.395; // big vertex
                camAim = true;
            } else if (gamepad2.dpad_left) { //far vertex
                launchStrength = 0.4667;
                //imuAim = true;
            } else if (gamepad2.b) {
                launchStrength = gamepad2.right_trigger;
            }

            CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();

            launchStrength *= CurrentVoltagePercentage;

            telemetry.addData("Volts", expansionHubVoltageSensor.getVoltage());
            telemetry.addData("percent of maxVolts", CurrentVoltagePercentage);

            leftLauncher.setPower(-launchStrength);
            rightLauncher.setPower(-launchStrength);

            if (launchStrength > 0.05) {
                telemetry.addData("Shooting", true);
                telemetry.addData("Launch Strength", launchStrength);
            } else {
                telemetry.addData("Shooting", false);
            }

            if (gamepad1.right_bumper){
                x = x * 0.25;
                y = y * 0.25;
                rx = rx * 0.25;
            }

            if (camAim){
                //rx = (rx * 0.25) + camera.getTagError(1) / 1000 * -1;
                //telemetry.addData("The error", camera.getTagError(1));

            }else if(imuAim){
                rx = (rx * 0.25) + (75 - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / -500;
                telemetry.addData("The error",(75 - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) );
            }



            if (DEBUG) {
                telemetry.addData("Y drive", y);
                telemetry.addData("X drive", x);
                telemetry.addData("RX drive", rx);
            }

            if (opt) {
                imu.resetYaw();
                telemetry.addData("Yaw reset", rx);
            }



            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (DEBUG) {telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));}

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (DEBUG) {
                telemetry.addData("ROTATED X", rotX);
                telemetry.addData("ROTATED Y", rotY);
            }

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (DEBUG) {
                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);
            }

            setPowerSafe(frontLeftMotor, frontLeftPower);
            setPowerSafe(backLeftMotor, backLeftPower);
            setPowerSafe(frontRightMotor, frontRightPower);
            setPowerSafe(backRightMotor, backRightPower);

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            DcMotor intake = hardwareMap.dcMotor.get("intake");
            if (a){
                setPowerSafe(intake, 1);
                //gamepad1.setLedColor(0,255,0,100);
                telemetry.addData("Intakeing", true);
            } else if (b) {
                setPowerSafe(intake, -1);
                //gamepad1.setLedColor(255,0,0,100);
                telemetry.addData("Outakeing", true);
            } else{
                //gamepad1.setLedColor(255,255,255,100);
                setPowerSafe(intake, 0);
            }

            //manip part
            boolean bumpL = gamepad2.left_bumper;
            boolean bumpR = gamepad2.right_bumper;
            boolean shift = gamepad2.a;

            if (bumpL && !lastBumpL) {
                target -= (1.0 / 3.0) * encoderPPR;
            }
            if (bumpR && !lastBumpR) {
                target += (1.0 / 3.0) * encoderPPR;
            }
            if (shift && !lastShift) {
                target += (1.0 / 6.0) * encoderPPR;
            }

            lastBumpL = bumpL;
            lastBumpR = bumpR;
            lastShift = shift;


            manipPID.setTarget(target + gameOffset);
            double manipPower = manipPID.calculateOutput(manipulator.getCurrentPosition());
            if(abs(manipPower) < 0.1){ manipPower = 0; } // no more EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
            if(!(abs(manipPID.getError(manipulator.getCurrentPosition())) < 2.5)) {
                manipulator.setPower(manipPower/1.5);
            }

            if (gamepad2.y){
                gameOffset += gamepad2.right_stick_x * 5;
            }

            if (DEBUG) {
                telemetry.addData("Manipulator Target", target);
                telemetry.addData("Manipulator Pos", manipulator.getCurrentPosition());
                telemetry.addData("Manipulator Power", manipPower);
                telemetry.addData("manip game offset", gameOffset);

                //camera.addLensTelemetry();
            }
            telemetry.update();
        }
    }

    public void setPowerSafe(DcMotor motor, double targetPower){
        final double SLEW_RATE = 0.2;
        double currentPower = motor.getPower();

        double desiredChange = targetPower - currentPower;
        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));

        motor.setPower(currentPower += limitedChange);
    }
}