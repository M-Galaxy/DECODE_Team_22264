package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Limelight {
    private Limelight3A limelight;
    public YawPitchRollAngles orientation;
    public double targetTag = 20;


    public Limelight (Limelight3A Lime, YawPitchRollAngles startOri){
        this.limelight = Lime;
        limelight.pipelineSwitch(0);
        this.orientation = startOri;
    }

    public void start() {
        limelight.start();
    }

    public void swichPipe(int index){
        limelight.pipelineSwitch(index);
    }

    public void setTargetTag(double targetTag) {
        this.targetTag = targetTag;

    }

    public LLResult GetResult(){
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return null;

        return result;
    }

    public int getObelisk() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag == null) continue;
            int id = (int) tag.getFiducialId();
            if (id == 21) return 1; // left
            if (id == 22) return 2; // center
            if (id == 23) return 3; // right
        }

        return 0; // none found
    }



    // Pass in the imu.getRobotYawPitchRollAngles()
    public double update(YawPitchRollAngles IMUOrientation){
        YawPitchRollAngles orientation = IMUOrientation;

        // MegaTag2 requires the robot's yaw to improve accuracy
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            // botpose_MT2 provides field-centric coordinates (X, Y, Z)
            Pose3D botpose = llresult.getBotpose_MT2();

//            telemetry.addData("Target Found", "Yes");
//            telemetry.addData("Tx", llresult.getTx());
//            telemetry.addData("Ty", llresult.getTy());

//            if (botpose != null) {
//                telemetry.addData("Botpose X", botpose.getPosition().x);
//                telemetry.addData("Botpose Y", botpose.getPosition().y);
//            }
            double x = botpose.getPosition().x; // Side-to-side offset
            double y = botpose.getPosition().y; // Vertical offset
            double z = botpose.getPosition().z; // Forward depth


            double distanceInMeters = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
            double distanceInInches = distanceInMeters * 39.3701;

            if (limelight.getLatestResult().getFiducialResults().get(0).getFiducialId() == targetTag) {
                return distanceInInches;
            }
            else{
                return 0;
            }
        } else {
            //telemetry.addData("Target Found", "No");
            return 0;
        }
    }

    public double getTargetXError() {
        LLResult result = limelight.getLatestResult();
        if (result == null || result.getFiducialResults().isEmpty()) return 0;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag == null) continue;

            if (tag.getFiducialId() == targetTag) {
                double tx = result.getTx(); // degrees

                double horizontalFOV = 59.6;
                double imageWidth = 320.0;

                return (tx / horizontalFOV) * imageWidth;
            }
        }
        return 0;
    }





}
