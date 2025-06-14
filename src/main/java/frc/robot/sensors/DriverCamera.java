package frc.robot.sensors;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class DriverCamera {
    private Thread thread;

    public DriverCamera() {

        thread = new Thread(()->{
            int WIDTH = 320/3;
            int HEIGHT = 240/3;
            //UsbCamera camera = CameraServer.startAutomaticCapture();
//            camera.setResolution(WIDTH, HEIGHT);
//            camera.setFPS(5);
//            camera.setBrightness(1);
//            camera.setExposureManual(10);

            CvSink sink = CameraServer.getVideo();
            CvSource source = CameraServer.putVideo("Driver", WIDTH, HEIGHT);

            Mat matIn = new Mat();
            Mat matOut = new Mat();
            Point p1 = new Point(HEIGHT-4,0);
            Point p2 = new Point(HEIGHT-4, 400);
            Point center = new Point(WIDTH/2,HEIGHT/2);
            Size s = new Size(HEIGHT, WIDTH);

            while(!thread.isInterrupted()) {
                if(sink.grabFrame(matIn) == 0){
                    source.notifyError("error");
                    continue;
                }
//                var rotationMatrix = Imgproc.getRotationMatrix2D(center, 270, 1);
//                Imgproc.warpAffine(matIn, matOut, rotationMatrix, s);
                Imgproc.line(matOut, p1, p2, new Scalar(0, 255, 0, 255),2, Imgproc.LINE_8);
                source.putFrame(matIn);
            }
        });
        thread.setDaemon(true);
        thread.start();
    }


}
