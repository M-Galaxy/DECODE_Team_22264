package org.firstinspires.ftc.teamcode.autos.Main;

import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Autonomous(name="RED - FAR - 6 BALL", group="0")
public class SixBallRed extends LinearOpMode {

    PIDController manipPID = new PIDController(0.008, 0, 0);


    private VoltageSensor expansionHubVoltageSensor;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timePerShot = new ElapsedTime();
    double timeLimit = 2;

    ElapsedTime TurnTmer = new ElapsedTime();

    public static final double MAX_LAUNCHER_RPM = 6000.0;

    // PID (RPM-based)
    public double kP = 0.0032;
    public double kI = 0.000001;
    public double kD = 0.0001;
    public double kF = 1.0 / MAX_LAUNCHER_RPM;


    private double integralSum = 0;
    private double lastError = 0;

    public static final double LAUNCHER_TPR = 28.0;


    @Override
    public void runOpMode() {

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx outL = hardwareMap.get(DcMotorEx.class, "outL");
        DcMotorEx outR = hardwareMap.get(DcMotorEx.class, "outR");

        // === IMU ===
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // === PINPOINT ===
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // === CAMERA ===
        Limelight camera = new Limelight(
                hardwareMap.get(Limelight3A.class, "limelight"),
                imu.getRobotYawPitchRollAngles()
        );

        // === DRIVETRAIN ===
        DcMotor lf = hardwareMap.dcMotor.get("LF");
        DcMotor lb = hardwareMap.dcMotor.get("LB");
        DcMotor rf = hardwareMap.dcMotor.get("RF");
        DcMotor rb = hardwareMap.dcMotor.get("RB");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === MANIPULATOR ===
        DcMotor manip = hardwareMap.dcMotor.get("bigWheel");
        manip.setDirection(DcMotorSimple.Direction.REVERSE);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manipPID.setTarget(manip.getCurrentPosition());

        // === SHOOTERS ===
        outL.setDirection(DcMotorSimple.Direction.REVERSE);

        expansionHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 1");
        double nominalVoltage = 13;
        // =========================
        // WAIT FOR START
        // =========================
        waitForStart();
        if (isStopRequested()) return;

        // reset sensors AFTER start
        imu.resetYaw();
        pinpoint.resetPosAndIMU();


        camera.setTargetTag(24);
        timer.reset();
        camera.start();

        // =========================
        // AUTO SEQUENCE
        // =========================


        // Get Obelisk detection
        //Update the camera's data

        int tagID = 21;

        camera.update(imu.getRobotYawPitchRollAngles());

        LLResult Result = camera.GetResult();
        int i = 0;
        while (i < 10) {
            Result = camera.GetResult();
            i += 1;
        }

        if (Result != null) {
            telemetry.addData("TAG OUTPUT", Result.getFiducialResults().get(0).getFiducialId());
            tagID = Result.getFiducialResults().get(0).getFiducialId();
        }

        // Wait 5 seconds so you can read it
        //sleep(5000);

        // save heading
        double startHeading =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180;

        outL.setPower(0.25);//I wanna rev em
        outR.setPower(0.25);// so much rev

        // drive forward 8 inches
        //driveForwardInches(2, 0.35, lf, lb, rf, rb, pinpoint);

        camera.swichPipe(2);//RED

//        if (camera.getTargetXError() == 0){
//            driveForwardInches(12, 0.5, lf, lb, rf, rb, pinpoint);
//            telemetry.addLine("I CANNOT SEE THE GOAL");
//            telemetry.update();
//            sleep(50000);
//            stop();
//        }


        // turn 20 deg CCW
        //turnToAngle(startHeading + 22.5, imu, lf, lb, rf, rb);
        TurnTmer.reset();
        double camer = 80;
        boolean withinTolerance = false;
        while ((TurnTmer.seconds() < 2.5) && opModeIsActive()) {// && (!withinTolerance)){// 3 seconds of correcting
            double rx = 0;
            camer = camera.getTargetXError();
            if (abs(camer) > 80) {// if far move at 50%
                if (camer > 0) {
                    rx = 0.5;
                } else {
                    rx = -0.5;
                }
            } else if (abs(camer) > 20) {// if close move at 25%
                if (camer > 0) {
                    rx = 0.25;
                } else {
                    rx = -0.25;
                }
            } else if (abs(camer) > 10) {// if close move at 25%
                if (camer > 0) {
                    rx = 0.125;
                } else {
                    rx = -0.125;
                }
            } else if (abs(camer) < 5) {
                withinTolerance = true;
            }
            // if close enough move at 0%
            lf.setPower(rx);
            lb.setPower(rx);
            rf.setPower(-rx);
            rb.setPower(-rx);
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);


        double launchStrength = 0.53;

        double CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();

        launchStrength *= CurrentVoltagePercentage;

        double rmpForFar = 2380;//2375

        // spin shooters
        waitTillReady(rmpForFar, outL, outR);

        // fire 3
        if (tagID == 21) {
            doManipAndShoot(manip, 1.0 / 6.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);sleep(300);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        }
        else if(tagID == 22){
            doManipAndShoot(manip, -1.0 / 6.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);sleep(300);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        } else if (tagID == 23) {
            doManipAndShoot(manip, -1.0 / 6.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);sleep(300);
            doManipAndShoot(manip, -1.0 / 3.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, -1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        }else{//I cant see the obbie but i still wanna score
            doManipAndShoot(manip, 1.0 / 6.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            sleep(250);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        }

        // stop everything
        outL.setPower(0);
        outR.setPower(0);

        // turn back to original heading
        turnToAngle(startHeading, imu, lf, lb, rf, rb); //FACE FORAWD

        outL.setPower(0);
        outR.setPower(0);

        //prep for intaking
        doManipAndShoot(manip, 1.0 / 6.0);
        intake.setPower(1);

        //driving to spike line
        // drive forward again 25 icnh
        driveForwardInches(12, 0.35, lf, lb, rf, rb, pinpoint);//0.4 16.5

        turnToAngle(startHeading - 90, imu, lf, lb, rf, rb); // turn to face spike RED

        //go pick up spike line
        driveForwardInches(27, 0.75, lf, lb, rf, rb, pinpoint);// this is moving  little

        //move to open slot
        doManipAndShoot(manip, 1.0 / 3.0);

        //wait and move to open slot
        sleep(100);
        doManipAndShoot(manip, 1.0 / 3.0);

        driveInches(-18, 0.75, lf, lb, rf, rb, pinpoint);//Return back to the center feild

        intake.setPower(0); //stop intake

        turnToAngle(startHeading, imu, lf, lb, rf, rb);// face forward

        driveInches(-10, 0.5, lf, lb, rf, rb, pinpoint);//Return to shooting line

        //driveInches(1,0.5, lf, lb, rf, rb, pinpoint);

        TurnTmer.reset();
        camer = 80;
        withinTolerance = false;
        while ((TurnTmer.seconds() < 2) && opModeIsActive()) {// && (!withinTolerance)){// 3 seconds of correcting
            double rx = 0;
            camer = camera.getTargetXError();
            if (abs(camer) > 80) {// if far move at 50%
                if (camer > 0) {
                    rx = 0.5;
                } else {
                    rx = -0.5;
                }
            } else if (abs(camer) > 20) {// if close move at 25%
                if (camer > 0) {
                    rx = 0.25;
                } else {
                    rx = -0.25;
                }
            } else if (abs(camer) > 10) {// if close move at 25%
                if (camer > 0) {
                    rx = 0.125;
                } else {
                    rx = -0.125;
                }
            } else if (abs(camer) < 5) {
                withinTolerance = true;
            }
            // if close enough move at 0%
            lf.setPower(rx);
            lb.setPower(rx);
            rf.setPower(-rx);
            rb.setPower(-rx);
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);



        waitTillReady(rmpForFar, outL, outR);

        // fire 3
        if (tagID == 21) {
            doManipAndShoot(manip, 1.0 / 6.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            sleep(100);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        } else if (tagID == 22) {
            doManipAndShoot(manip, -1.0 / 6.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);
            sleep(100);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        } else if (tagID == 23) {
            doManipAndShoot(manip, -1.0 / 6.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);
            sleep(100);
            doManipAndShoot(manip, -1.0 / 3.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, -1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        }else{//I cant see the obbie but i still wanna score
            doManipAndShoot(manip, 1.0 / 6.0, outL, outR, rmpForFar); //Green
            waitTillReady(rmpForFar, outL, outR);
            sleep(100);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            waitTillReady(rmpForFar, outL, outR);
            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
            //sleep(500);
            waitTillReady(rmpForFar, outL, outR);
        }

        // drive forward again 10 inch
        driveForwardInches(12, 0.5, lf, lb, rf, rb, pinpoint);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    void waitTillReady(double targetRPM, DcMotorEx outL, DcMotorEx outR) {
        timePerShot.reset();
        double averageRPM = 0;
        while ((timePerShot.seconds() < 4.20) && !(abs(targetRPM - averageRPM) < 25) && opModeIsActive()) {
            double power = returnPID(targetRPM, averageRPM);

            outL.setPower(power);
            outR.setPower(power);

            double velL = outL.getVelocity();
            double velR = outR.getVelocity();
            double avgTicksPerSec = (velL + velR) / 2.0;
            telemetry.update();

            averageRPM = (avgTicksPerSec / LAUNCHER_TPR) * 60.0;
        }
    }

    void spinTheLaunchers(double targetRPM, DcMotorEx outL, DcMotorEx outR) {
        double velL = outL.getVelocity();
        double velR = outR.getVelocity();
        double avgTicksPerSec = (velL + velR) / 2.0;
        double averageRPM = (avgTicksPerSec / LAUNCHER_TPR) * 60.0;

        double power = returnPID(targetRPM, averageRPM);


        telemetry.addData("Average RPM", averageRPM);
        telemetry.addData("Target RPM``", targetRPM);

        outL.setPower(power);
        outR.setPower(power);
    }

    // =====================================================
    // DRIVE USING PINPOINT Y
    // =====================================================
    void driveForwardInches(
            double inches,
            double power,
            DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
            GoBildaPinpointDriver pinpoint
    ) {
        //pinpoint.resetPosAndIMU();



        for (int i = 0; i < 10; i++) {
            pinpoint.update();
        }

        pinpoint.update();

        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);

        long startTime = System.currentTimeMillis();
        long timeoutMs = 3000;

        while (opModeIsActive()) {

            pinpoint.update();
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            double traveled = Math.abs(currentY - startY);

            telemetry.addData("Traveled", traveled);
            telemetry.addData("Target", inches);
            telemetry.update();

            if (traveled >= inches) break;
            if (System.currentTimeMillis() - startTime > timeoutMs) break;

            double speed = power;

            if (traveled > inches * 0.5) {
                speed = power * 0.6;
            }

            lf.setPower(speed);
            lb.setPower(speed);
            rf.setPower(speed);
            rb.setPower(speed);
        }

        // Fine correction
        pinpoint.update();
        double error = inches - Math.abs(
                pinpoint.getPosition().getY(DistanceUnit.INCH) - startY
        );

        startTime = System.currentTimeMillis();
        timeoutMs = 2000;

        while (opModeIsActive() && Math.abs(error) > 0.2 && Math.abs(error) < 2) {

            double correction = Math.signum(error) * 0.15;

            if (System.currentTimeMillis() - startTime > timeoutMs) break;


            lf.setPower(correction);
            lb.setPower(correction);
            rf.setPower(correction);
            rb.setPower(correction);

            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.addData("Target", inches);
            telemetry.update();

            pinpoint.update();
            error = inches - Math.abs(
                    pinpoint.getPosition().getY(DistanceUnit.INCH) - startY
            );
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }


    // =====================================================
    // TURN USING IMU
    // =====================================================
    void turnToAngle(
            double targetDeg,
            IMU imu,
            DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb
    ) {
        double kP = 0.0025;// 0.01;
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(targetDeg - yaw);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetDeg);
            telemetry.addData("Error", error);
            telemetry.update();

            if (Math.abs(error) < 1.0) break;
            if (System.currentTimeMillis() - start > 2000) break;

            double power = error * kP;
            power = Math.max(-0.25, Math.min(0.25, power));

            lf.setPower(power);
            lb.setPower(power);
            rf.setPower(-power);
            rb.setPower(-power);
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }


    // =====================================================
    // SHOOTER MANIP
    // =====================================================
    void doManipAndShoot(DcMotor manip, double revolutions, DcMotorEx outL, DcMotorEx outR, double targetRPM) {
        double ticks = revolutions * 1075.4; // 537.7 * 2
        double target = manip.getCurrentPosition() + ticks;
        manipPID.setTarget(target);

        long t0 = System.currentTimeMillis();

        while (opModeIsActive()
                && Math.abs(manipPID.getError(manip.getCurrentPosition())) > 0.25
                && System.currentTimeMillis() - t0 < 2000) {//1500

            double output = manipPID.calculateOutput(manip.getCurrentPosition()) * 0.4;

            // Enforce a minimum power to overcome static friction for small moves
            if (Math.abs(ticks) < 300) { // roughly 1/6 rev or less
                output = Math.signum(output) * Math.max(Math.abs(output), 0.2);
            }

            spinTheLaunchers(targetRPM, outL, outR);

            manip.setPower(output);
            telemetry.update();
        }

        manip.setPower(0);
        sleep(250);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double returnPID(double reference, double state) {
        double error = reference - state;
        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;
        timer.reset();

        double pOut = kP * error;
        integralSum += error * dt;
        if (Math.abs(error) > 500) integralSum = 0;
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


    // =====================================================
    // SHOOTER MANIP
    // =====================================================
    void doManipAndShoot(DcMotor manip, double revolutions) {
        double ticks = revolutions * 1075.4; // 537.7 * 2
        double target = manip.getCurrentPosition() + ticks;
        manipPID.setTarget(target);

        long t0 = System.currentTimeMillis();

        while (opModeIsActive()
                && Math.abs(manipPID.getError(manip.getCurrentPosition())) > 0.25
                && System.currentTimeMillis() - t0 < 1500) {

            double output = manipPID.calculateOutput(manip.getCurrentPosition()) * 0.4;

            // Enforce a minimum power to overcome static friction for small moves
            if (Math.abs(ticks) < 200) { // roughly 1/6 rev or less
                output = Math.signum(output) * Math.max(Math.abs(output), 0.2);
            }

            manip.setPower(output);
        }

        manip.setPower(0);
        //sleep(250);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void driveInches(
            double inches,
            double power,
            DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
            GoBildaPinpointDriver pinpoint
    ) {

        double direction = Math.signum(inches);
        double targetDistance = Math.abs(inches);
        power = Math.abs(power) * direction;

        // Let pinpoint stabilize
        for (int i = 0; i < 10; i++) {
            pinpoint.update();
        }

        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);

        long startTime = System.currentTimeMillis();
        long timeoutMs = 3000;

        while (opModeIsActive()) {

            pinpoint.update();
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            double traveled = Math.abs(currentY - startY);

            telemetry.addData("Traveled", traveled);
            telemetry.addData("Target", targetDistance);
            telemetry.update();

            if (traveled >= targetDistance) break;
            if (System.currentTimeMillis() - startTime > timeoutMs) break;

            double speed = power;

            if (traveled > targetDistance * 0.5) {
                speed = power * 0.6;
            }

            lf.setPower(speed);
            lb.setPower(speed);
            rf.setPower(speed);
            rb.setPower(speed);
        }

        // Fine correction (same safety guard as new auto)
        pinpoint.update();
        double error = targetDistance - Math.abs(
                pinpoint.getPosition().getY(DistanceUnit.INCH) - startY
        );

        while (opModeIsActive() && Math.abs(error) > 0.2 && Math.abs(error) < 2) {

            double correction = Math.signum(error) * 0.15 * direction;

            lf.setPower(correction);
            lb.setPower(correction);
            rf.setPower(correction);
            rb.setPower(correction);

            telemetry.addData("error", error);
            telemetry.update();

            pinpoint.update();
            error = targetDistance - Math.abs(
                    pinpoint.getPosition().getY(DistanceUnit.INCH) - startY
            );
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }



}
