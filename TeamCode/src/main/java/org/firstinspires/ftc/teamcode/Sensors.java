
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.List;
public class Sensors extends NewRobotTeleopMecanum{

    private FtcDashboard dashboard;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    TelemetryPacket initPacket = new TelemetryPacket();


    public void addTele(TelemetryPacket packet, String caption, Object data){
        data = data.toString();
        telemetry.addData(caption, data);
        packet.put(caption, data);
        packet.put("caption"," data");
    }

    public void initCam() {

        // Create the AprilTag processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        DashboardCameraStreamProcessor streamProcessor =
                new DashboardCameraStreamProcessor();

        dashboard = FtcDashboard.getInstance();

        // Initialize the webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                        WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(streamProcessor)
                .build();

        FtcDashboard.getInstance()
                .startCameraStream(streamProcessor, 0);

        addTele(initPacket, "Camera initializing...", "");

        telemetry.update();
    }
    public void runCam(){
        TelemetryPacket mainPacket = new TelemetryPacket();

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        // Basic system information
        addTele(mainPacket,"Status", "Running");
        addTele(mainPacket,"Camera State", visionPortal.getCameraState().toString());
        addTele(mainPacket,"Total Detections", detections.size());


        int index = 0;

        for (AprilTagDetection detection : detections) {

            telemetry.addData(
                    "Detection " + index + " Type",
                    detection.getClass().getSimpleName()
            );

            mainPacket.put(
                    "Detection " + index + " Type",
                    detection.getClass().getSimpleName()
            );

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                telemetry.addData(
                        "Cluster " + index,
                        cluster.metadata.name
                );

                mainPacket.put(
                        "Cluster " + index,
                        cluster.metadata.name
                );
                addTele(mainPacket,"Range", detection.ftcPose.range);
                addTele(mainPacket,"Bearing", detection.ftcPose.bearing);
                addTele(mainPacket,"Yaw", detection.ftcPose.yaw);

            } else if (detection instanceof AprilTagSingleDetection) {

                AprilTagSingleDetection tag =
                        (AprilTagSingleDetection) detection;

                addTele(
                        mainPacket,
                        "Tag " + index + " ID",
                        tag.id
                );
                addTele(mainPacket,"Range", detection.ftcPose.range);
                addTele(mainPacket,"Bearing", detection.ftcPose.bearing);
                addTele(mainPacket,"Yaw", detection.ftcPose.yaw);
            }

            index++;
        }

        telemetry.update();
        dashboard.sendTelemetryPacket(mainPacket);
    }
    public void close() {
        dashboard.stopCameraStream();
        visionPortal.close();
    }


}