package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class VisionTester extends OpMode {
    VisionBrain visionBrain;


    @Override
    public void init() {
        visionBrain = new VisionBrain();
        visionBrain.useWebCam = true;
        visionBrain.showCamera = false; // useful for sighting on phone only
        visionBrain.showCameraOD = true; // useful for seeing object detection on phone only
        visionBrain.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
        visionBrain.init(this, telemetry);
        visionBrain.activate();
        // visionBrain.initTfod();
        // visionBrain.initVuforia();
    }

    @Override
    public void loop() {
        //visionBrain.process(2000);
        //teleDrive();
    }
}
