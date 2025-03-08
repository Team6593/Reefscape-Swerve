// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Camera {
    int videoPort;
    String cameraName;

    public Camera(int videoPort, String cameraName) {
      this.videoPort = videoPort;
      this.cameraName = cameraName;
      
    }

    Thread streamThread;

  public void streamVideo() {
    streamThread = new Thread(
      () -> {
        var camera = CameraServer.startAutomaticCapture(videoPort);
        var cameraWidth = 320;
        var cameraHeight = 240;
        //camera.setPixelFormat(PixelFormat.kGray);

        camera.setResolution(cameraWidth, cameraHeight);
        camera.setFPS(20);

        var red = new Scalar(0, 0, 255);
        var crosshairPoint = new Point();


        var cvSink = CameraServer.getVideo();
        var outputStream = CameraServer.putVideo(cameraName,
         cameraWidth, cameraHeight);
        
         // mats are memory expensive, it's best to just use one
        var mat = new Mat();
        // this can never be true the robot must be off for this to be true
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }

          crosshairPoint.x = 90;
          crosshairPoint.y = 100;
          Imgproc.circle(mat, crosshairPoint, 40, red);

          long deltaTime = camera.getLastFrameTime();
          SmartDashboard.putNumber("DriverStation Camera delay", deltaTime);
          outputStream.putFrame(mat);
        }

      });
      streamThread.setDaemon(true);
      streamThread.start();
  }
}
