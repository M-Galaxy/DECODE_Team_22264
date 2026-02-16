package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Configurable
@TeleOp(name="State Ready Code - Decode Blue", group="00 - current testing scripts")
public class StateReadyTeleop extends LinearOpMode {

    // =====================
    // DEBUG FLAGS
    // =====================
    final boolean DEBUG = false;
    final boolean LOCATIONDATA = false;

    // =====================
    // ENCODER CONSTANTS & TUNING (TUNE HERE)
    // =====================
    public static final double LAUNCHER_TPR = 28.0;            // high-speed launcher encoder ticks per rev
    public static final double MANIPULATOR_TPR = 537.7 * 2;   // slow manipulator encoder ticks per rev

    public static final double MAX_LAUNCHER_RPM = 6000.0;     // hardware max RPM

    // PID (RPM-based)
    public static double kP = 0.0032;
    public static double kI = 0.000001;
    public static double kD = 0.0001;

    //Manip PID
    public static double manipKp = 0.008;
    public static double manipKi = 0.0;
    public static double manipKd = 0.0;
    public static double manipKf = 0; //1;

    // Feedforward (≈ 1 / max RPM)
    public static double kF = 1.0 / MAX_LAUNCHER_RPM;

    // Voltage compensation
    public static final double NOMINAL_VOLTAGE = 12.38;

    // =====================
    // STATE VARIABLES
    // =====================
    private VoltageSensor expansionHubVoltageSensor;
    private ElapsedTime timer = new ElapsedTime();

    private ElapsedTime MANtimer = new ElapsedTime();

    private ElapsedTime lunchTimer = new ElapsedTime();

    private double integralSum = 0;
    private double lastError = 0;

    private double manipIntegralSum = 0;
    private double manipLastError = 0;

    private double lastTimeNano;
    private double lastEncoderPositionL;
    private double lastEncoderPositionR;

    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;

    private TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // =====================
        // DRIVE MOTORS
        // =====================
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =====================
        // IMU
        // =====================
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // =====================
        // LAUNCHER MOTORS
        // =====================
        expansionHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 1");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "outL");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "outR");

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTimeNano = System.nanoTime();
        lastEncoderPositionL = leftLauncher.getCurrentPosition();
        lastEncoderPositionR = rightLauncher.getCurrentPosition();

        // =====================
        // MANIPULATOR MOTOR
        // =====================
        DcMotor manipulator = hardwareMap.dcMotor.get("bigWheel");
        manipulator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manipulator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manipulator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manipulator.setDirection(DcMotorSimple.Direction.REVERSE);




        double target = manipulator.getCurrentPosition();

        boolean lastBumpL = false;
        boolean lastBumpR = false;
        boolean lastShift = false;
        boolean lastTurnIfReady = false;
        double gameOffset = 0;

        DistanceSensor distSens = hardwareMap.get(DistanceSensor.class, "distance");

        ElapsedTime distanceSensorTime = new ElapsedTime();

        boolean hasRot = false;

        // =====================
        // CAMERA
        // =====================
        Limelight camera = new Limelight(hardwareMap.get(Limelight3A.class, "limelight"),
                imu.getRobotYawPitchRollAngles());
        camera.setTargetTag(20); // BLUE
        camera.swichPipe(1);

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        waitForStart();
        timer.reset();
        MANtimer.reset();
        lunchTimer.reset();
        distanceSensorTime.reset();

        camera.start();

        int targetRPM = 0;

        while (opModeIsActive()) {

            //PIDController manipPID = new PIDController(manipKp, manipKi, manipKd);//0.0075

            // CAMERA STUFF
            camera.update(imu.getRobotYawPitchRollAngles());
            LLResult Result = camera.GetResult();





            // =====================
            // GAMEPAD INPUTS
            // =====================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean opt = gamepad1.options;

            boolean camAim = gamepad1.dpad_down;

            boolean slowMode = gamepad1.right_bumper;

            boolean intaking = gamepad1.a;

            boolean inTake = gamepad1.y;
            boolean outTake = gamepad1.b;

            double launchStrength = 0;
            targetRPM = 0;
            if(gamepad2.dpad_right) {
                targetRPM = 1750;
            } else if(gamepad2.dpad_down) {
                launchStrength = 0.31;//TODO - remove the extra ones
            } else if(gamepad2.dpad_up) {
                targetRPM = 1990;
            } else if(gamepad2.dpad_left) {
                targetRPM = 2250;
            } else if(gamepad2.b) {
                launchStrength = gamepad2.right_trigger;
                targetRPM = (int)(gamepad2.right_trigger * 6000);
            }

            boolean bumpL = gamepad2.left_bumper;
            boolean bumpR = gamepad2.right_bumper;
            boolean shift = gamepad2.a;

            boolean manipOffsetToggle = gamepad2.y;
            double manipOffsetAnalog = gamepad2.right_stick_x;

            boolean turnIfReady = gamepad2.x;

            // =====================
            // LAUNCHER RPM CALCULATION
            // =====================
            double velL = leftLauncher.getVelocity();
            double velR = rightLauncher.getVelocity();
            double avgTicksPerSec = (velL + velR) / 2.0;
            double averageRPM = (avgTicksPerSec / LAUNCHER_TPR) * 60.0;

            // =====================
            // VOLTAGE COMPENSATION
            // =====================
            double batteryVoltage = expansionHubVoltageSensor.getVoltage();
            double voltagePercent = NOMINAL_VOLTAGE / batteryVoltage;

            // =====================
            // PID CONTROL + FEEDFORWARD
            // =====================
            double pidPower = returnPID(targetRPM, averageRPM);
            double compensatedPower = pidPower * voltagePercent;
            compensatedPower = Math.max(0.0, Math.min(1.0, compensatedPower));

            if (compensatedPower > 0.01) {
                leftLauncher.setPower(compensatedPower);
                rightLauncher.setPower(compensatedPower);
            }
            else {
                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
            }


            // =====================
            // TELEMETRY
            // =====================
            if(DEBUG) {
                telemetry.addData("Launcher RPM", averageRPM);
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Launcher Power", compensatedPower);
                telemetry.addData("Manipulator Pos", manipulator.getCurrentPosition());
                telemetry.addData("Manipulator Target", target + gameOffset);

                panelsTelemetry.addData("Target RPM", targetRPM);
                panelsTelemetry.addData("Current RPM", averageRPM);


                panelsTelemetry.addData("Camera Error", camera.getTargetXError());
                telemetry.addData("Camera Error", camera.getTargetXError());

                panelsTelemetry.update();
            }

            boolean lunchReady = (abs(targetRPM - averageRPM) < 20);

            if(lunchReady) {
                telemetry.addLine("READY FOR LUNCH");
            }

            telemetry.update();

            // =====================
            // DRIVE CONTROL
            // =====================
            if (slowMode) { x *= 0.25; y *= 0.25; rx *= 0.25; }


            if(camAim){
                double camer = camera.getTargetXError();
                if(abs(camer) > 80){// if far move at 50%
                    if(camer > 0){
                        rx = 0.5;
                    }else{
                        rx = -0.5;
                    }
                }else if(abs(camer) > 20) {// if close move at 25%
                    if(camer > 0){
                        rx = 0.25;
                    }else{
                        rx = -0.25;
                    }
                }// if close enough move at 0%
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1; // strafing compensation
            double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);

            if(opt) imu.resetYaw();


            // =====================
            // INTAKE & AUTO INTAKE MODE
            // =====================

            if(inTake || intaking){intake.setPower(1);}// intaking = true;}
            else if(outTake){ intake.setPower(-1);}// intaking = false;}
            else intake.setPower(0);


            double distCm = distSens.getDistance(DistanceUnit.CM);

            if(intaking){
                if (distanceSensorTime.seconds() > 1){//IF IS NOT CURRENTLY ROTATING
                    if (distCm > 15){ //IF I CANNOT SEE A BALL
                        telemetry.addLine("WAITING FOR BALL");
                    } else if (6 < distCm && distCm < 15) { //IF I CAN SEE A BALL IN THE MIDDLE
                        telemetry.addLine("START ROTATING");
                        target += (1.0/3.0) * MANIPULATOR_TPR;//ROTATE THE WHEEL
                        distanceSensorTime.reset();//START CURRENTLY ROTATING TIMER
                    }
                }else{//IF WE ARE STILL WAITING FOR IT TO STOP SPINNING
                    telemetry.addLine("ROTATING");
                }
            }


            // =====================
            // MANIPULATOR CONTROL
            // =====================

            if(bumpL && !lastBumpL) target -= (1.0/3.0) * MANIPULATOR_TPR;
            if(bumpR && !lastBumpR) target += (1.0/3.0) * MANIPULATOR_TPR;
            if(shift && !lastShift){ target += (1.0/6.0) * MANIPULATOR_TPR; lunchTimer.reset();}

            if (turnIfReady && !lastTurnIfReady){
                target += (1.0/6.0);
            }
            if(turnIfReady && lunchReady && (lunchTimer.seconds() > 2)){
                target += (1.0/3.0) * MANIPULATOR_TPR;
                lunchTimer.reset();
            }

            if (lunchTimer.seconds() > 20){
                lunchTimer.reset();
            }
            if (timer.seconds() > 20){
                timer.reset();
            }


            lastBumpL = bumpL; lastBumpR = bumpR; lastShift = shift; lastTurnIfReady = turnIfReady;

            double manipPower = manipPID(manipulator.getCurrentPosition(),target + gameOffset);
            //manipulator.setZeroPowerBehavior(abs(manipPower) < 0.1 ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
            manipulator.setPower(-manipPower);
            if(manipOffsetToggle) gameOffset += manipOffsetAnalog * 5;

            if(DEBUG) {
                panelsTelemetry.addData("Target Manip Position", target + gameOffset);
                panelsTelemetry.addData("Current Position", manipulator.getCurrentPosition());
                panelsTelemetry.addData("manip Power", manipPower);
                telemetry.addData("MANIP TIMER", distanceSensorTime.seconds());
                panelsTelemetry.addData("Distance CM", distCm);
                telemetry.addData("Distance CM", distCm);
            }

        }
    }

    // =====================
    // PID FOR LAUNCHER
    // =====================
    public double returnPID(double reference, double state) {
        double error = reference - state;
        double dt = timer.seconds();
        if(dt <= 0) dt = 0.001;
        timer.reset();

        double pOut = kP * error;
        integralSum += error * dt;
        if(Math.abs(error) > 500) integralSum = 0;
        double iOut = kI * integralSum;
        double derivative = (error - lastError) / dt;
        double dOut = kD * derivative;
        double fOut = kF * reference;
        lastError = error;
        //if(Math.abs(error) < 20) {
        //    return reference;
        //}
        return pOut + iOut + dOut + fOut;
    }


    // =====================
    // PID FOR MANIPULATOR
    // =====================
    public double manipPID(double reference, double state) {
        double error = reference - state;
        double dt = MANtimer.seconds();
        if(dt <= 0) dt = 0.001;
        MANtimer.reset();

        double pOut = manipKp * error;
        manipIntegralSum += error * dt;
        if(Math.abs(error) > 500) integralSum = 0;
        double iOut = manipKi * integralSum;
        double derivative = (error - lastError) / dt;
        double dOut = manipKd * derivative;
        double fOut = manipKf * reference;
        manipLastError = error;

        return pOut + iOut + dOut + fOut;
    }
}
