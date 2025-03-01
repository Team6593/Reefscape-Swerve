// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Camera {

    public Camera() {
    }

    Thread streamThread;

  public void streamVideo() {
    streamThread = new Thread(
      () -> {
        var camera = CameraServer.startAutomaticCapture(1);
        var cameraWidth = 320;
        var cameraHeight = 240;
        //camera.setPixelFormat(PixelFormat.kGray);

        camera.setResolution(cameraWidth, cameraHeight);
        camera.setFPS(30);

        var cvSink = CameraServer.getVideo();
        var outputStream = CameraServer.putVideo("FishEye",
         cameraWidth, cameraHeight);
        
         // mats are memory expensive, it's best to just use one
        var mat = new Mat();
        // this can never be true the robot must be off for this to be true
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          long deltaTime = camera.getLastFrameTime();
          SmartDashboard.putNumber("DriverStation Camera delay", deltaTime);
          outputStream.putFrame(mat);
        }

      });
      streamThread.setDaemon(true);
      streamThread.start();
  }
}
