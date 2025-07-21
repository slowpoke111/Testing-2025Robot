package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayOutputStream;
import java.util.function.Supplier;

import javax.imageio.ImageIO;

import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionSubsystem extends SubsystemBase {
  private HttpCamera camera;
  private CvSink cvSink;

  private final VisionSystemSim visionSim = new VisionSystemSim("main");
  private final PhotonCameraSim cameraSim;
  private final LEDSubsystem leds; // Assuming you have an LED subsystem for status indication
  private final Supplier<Pose2d> robotPoseSupplier; // Supplier for the robot's pose, if needed

  public VisionSubsystem(LEDSubsystem leds, Supplier<Pose2d> robotPoseSupplier) {
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    visionSim.addAprilTags(tagLayout);

    this.leds = leds;
    this.robotPoseSupplier = robotPoseSupplier;


    SimCameraProperties cameraProps = new SimCameraProperties();
    cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(90));
    cameraProps.setFPS(20);
    cameraProps.setAvgLatencyMs(30);
    cameraProps.setLatencyStdDevMs(5);
    cameraProps.setCalibError(0.3, 0.05);

    PhotonCamera camera1 = new PhotonCamera("TestCam");
    cameraSim = new PhotonCameraSim(camera1, cameraProps);

    Transform3d robotToCamera = new Transform3d(
      new Translation3d(0, 0, 1),           // 1m above ground
      new Rotation3d(0, 0.5, 0)             // Slight pitch forward
    );
    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableRawStream(true);
    cameraSim.enableDrawWireframe(true); // Optional: expensive

    // Initialize camera connection to the external stream
    camera = new HttpCamera("stream", "http://localhost:1181/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer);
    CameraServer.addCamera(camera);
    cvSink = CameraServer.getVideo(camera);
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  @Override
  public void periodic() {
    // Simulate a static robot pose (only needed once if no motion)
    visionSim.update(robotPoseSupplier.get());
  }

  public BufferedImage getImage() {
    Mat mat = new Mat();
    if (cvSink.grabFrame(mat) == 0) {
      System.err.println("Error getting frame: " + cvSink.getError());
      return null;
    }

    return matToBufferedImage(mat);
  }

  private BufferedImage matToBufferedImage(Mat mat) {
    try {
      Mat converted = new Mat();
      if (mat.channels() == 1) {
        Imgproc.cvtColor(mat, converted, Imgproc.COLOR_GRAY2BGR);
      } else {
        Imgproc.cvtColor(mat, converted, Imgproc.COLOR_RGB2BGR);
      }

      int width = converted.width();
      int height = converted.height();
      byte[] data = new byte[width * height * (int)converted.elemSize()];
      converted.get(0, 0, data);

      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
      final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
      System.arraycopy(data, 0, targetPixels, 0, data.length);

      return image;
    } catch (Exception e) {
      System.err.println("Error converting Mat to BufferedImage: " + e.getMessage());
      e.printStackTrace();
      return null;
    }
  }

  public byte[] getImageBytes() {
    try {
      BufferedImage image = getImage();
      if (image == null) {
        System.err.println("Failed to retrieve image from camera");
        return new byte[0];
      }

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      ImageIO.write(image, "jpg", outputStream);
      return outputStream.toByteArray();
    } catch (Exception e) {
      System.err.println("Error converting image to bytes: " + e.getMessage());
      e.printStackTrace();
      return new byte[0];
    }
  }
}
