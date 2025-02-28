package frc.robot.sensors;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class DriverCamera {
    private Thread thread;

    public DriverCamera() {

        thread = new Thread(()->{
            int WIDTH = 320/3;
            int HEIGHT = 240/3;
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(WIDTH, HEIGHT);
            camera.setFPS(5);
            camera.setBrightness(1);
            camera.setExposureManual(10);

            CvSink sink = CameraServer.getVideo();
            CvSource source = CameraServer.putVideo("Driver", WIDTH, HEIGHT);

            Mat mat = new Mat();
            Point p1 = new Point(HEIGHT-4,0);
            Point p2 = new Point(HEIGHT-4, 400);

            while(!thread.isInterrupted()) {
                if(sink.grabFrame(mat) == 0){
                    source.notifyError("error");
                    continue;
                }
                Imgproc.line(mat, p1, p2, new Scalar(0, 255, 0, 255),2, Imgproc.LINE_8);
                source.putFrame(mat);
            }
        });
        thread.setDaemon(true);
        thread.start();
    }


}
