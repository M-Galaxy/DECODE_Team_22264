package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Launcher {

    final double encoderResolutionTPR = 28.0;
    final double MaxRPM = 4500.0;

    DcMotor Left;
    DcMotor Right;

    double lastEncoderPositionL;
    double lastEncoderPositionR;
    long lastTimeNano;

    public Launcher(DcMotor left, DcMotor right) {
        this.Left = left;
        this.Right = right;

        Left.setDirection(DcMotorSimple.Direction.REVERSE);

        lastEncoderPositionL = Left.getCurrentPosition();
        lastEncoderPositionR = Right.getCurrentPosition();

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastTimeNano = System.nanoTime();
    }

    public double getRPM() {
        long now = System.nanoTime();
        double dt = (now - lastTimeNano) / 1e9;
        if (dt <= 0) return 0;

        double currentL = Left.getCurrentPosition();
        double currentR = Right.getCurrentPosition();

        double deltaL = currentL - lastEncoderPositionL;
        double deltaR = currentR - lastEncoderPositionR;

        lastEncoderPositionL = currentL;
        lastEncoderPositionR = currentR;
        lastTimeNano = now;

        double leftRPM = (deltaL / encoderResolutionTPR) / dt * 60.0;
        double rightRPM = (deltaR / encoderResolutionTPR) / dt * 60.0;

        return (leftRPM + rightRPM) / 2.0;
    }

    public double powerFromRPM(double rpm) {
        return rpm / MaxRPM;
    }

    public void RPMatPowerTest(double power) {
        Left.setPower(power);
        Right.setPower(power);
    }

    public void maintainRPM(double targetRPM) {
        double measuredRPM = getRPM();
        double error = targetRPM - measuredRPM;

        double power = powerFromRPM(targetRPM) + error * 0.0002;
        power = Math.max(0.0, Math.min(1.0, power));

        Left.setPower(power);
        Right.setPower(power);
    }

    public void STOP() {
        Left.setPower(0);
        Right.setPower(0);
    }

    public double getLeftEncoder() {
        return Left.getCurrentPosition();
    }

    public double getRightEncoder() {
        return Right.getCurrentPosition();
    }
}
