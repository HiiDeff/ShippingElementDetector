import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

public class Detector {

    // Webcam constants:
    private static final String WEBCAM_CONFIG_NAME = "Webcam 1";
    private static final String TAG = "Webcam";
    private static final int TIMEOUT = Integer.MAX_VALUE;

    // Color constants:
    private static final double COLOR_DIV = 255.0;
    private static final double EPSILON = 1e-5;

    // Image processing constants:
    private static final int THRESHOLD = 500; // Any amount of red/blue pixels greater than THRESHOLD is considered as a (visible) marker.

    // Webcam variables:
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private Handler callbackHandler;

    // Image processing variables:
    private ElementPosition elementPosition = ElementPosition.NONE;
    private final int left;
    private final int right;

    public Detector(HardwareMap hardwareMap) {
        this(hardwareMap, 0, 640);
    }

    public Detector(HardwareMap hardwareMap, int left, int right) {
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, WEBCAM_CONFIG_NAME);
        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        openCamera();
        if (camera != null) startCamera();

        this.left = left;
        this.right = right;
    }

    public void loadImage() {
        Bitmap bmp = frameQueue.poll();
        if (bmp != null) {
            elementPosition = saveBitmap(bmp);
            bmp.recycle();
        }
    }

    public ElementPosition getElementPosition() {
        return elementPosition;
    }

    public void shutDown() {
        // Stops the camera:
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
        // Closes the camera:
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    private void initializeFrameQueue(int capacity) {
        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(capacity));
        frameQueue.setEvictAction(Bitmap::recycle);
    }

    private void openCamera() {
        if (camera != null) return;
        Deadline deadline = new Deadline(TIMEOUT, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return;
        final int imageFormat = ImageFormat.YUY2;
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) return;
        final Size size = new Size(640, 480);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                (session1, request, cameraFrame) -> {
                                    Bitmap bmp = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(bmp);
                                    frameQueue.offer(bmp);
                                },
                                Continuation.create(callbackHandler, (session12, cameraCaptureSequenceId, lastFrameNumber) ->
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber))
                        );
                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            synchronizer.finish(null);
        }
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        cameraCaptureSession = synchronizer.getValue();
    }

    private ElementPosition saveBitmap(Bitmap bitmap) {
        Bitmap newBitmap = Bitmap.createBitmap(bitmap, left, 120, right - left, 240);
        int leftCount = 0;
        int rightCount = 0;
        for (int j = 0; j < 240; j++) {
            StringBuilder colorBuilder = new StringBuilder();
            for (int i = 0; i < right - left; i++) {
                boolean isBlueOrRed = isBlueOrRed(newBitmap.getPixel(i, j), colorBuilder);
                if (isBlueOrRed) {
                    if (i < (left + right) / 2) {
                        leftCount++;
                    } else {
                        rightCount++;
                    }
                }
            }
            // Logs the found hues to the logcat:
            RobotLog.e("row: %d, map: %s", j, colorBuilder.toString());
        }
        // Saves the image to local storage for finding a good camera placement:
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                newBitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
        }
        if (leftCount > THRESHOLD && rightCount > THRESHOLD) {
            return ElementPosition.NONE;
        } else if (leftCount > THRESHOLD) {
            return ElementPosition.RIGHT;
        } else if (rightCount > THRESHOLD) {
            return ElementPosition.LEFT;
        } else {
            // Returns a random position, for some reason the camera detected no uncovered markers:
            int rand = (int) (Math.random() * 2);
            if (rand == 0) return ElementPosition.LEFT;
            return ElementPosition.RIGHT;
        }
    }

    private static boolean isBlueOrRed(int color, StringBuilder builder) {
        // Converts color from RGB565 format to HSV format:
        double r = Color.red(color) / COLOR_DIV;
        double g = Color.green(color) / COLOR_DIV;
        double b = Color.blue(color) / COLOR_DIV;
        double cmax = max(r, g, b);
        double cmin = -max(-r, -g, -b);
        double diff = cmax - cmin;
        double h, s, v;
        // Finds h:
        if (epsilonEquals(cmax, cmin)) h = 0;
        else if (cmax == r) h = (60 * ((g - b) / diff) + 360) % 360;
        else if (cmax == g) h = (60 * ((b - r) / diff) + 120) % 360;
        else h = (60 * ((r - g) / diff) + 240) % 360;
        // Finds s:
        if (epsilonEquals(cmax, 0)) s = 0;
        else s = (diff / cmax) * 100;
        // Finds v:
        v = cmax * 100;
        // Returns if the color falls within the range considered as orange:
        if (s < 40) builder.append(" ");
        else builder.append((int) (h * 10 / 256));
        return s > 30 && (h > 200 && h < 260 || h < 20 || h > 340);
    }

    private static boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    private static double max(double... nums) {
        double max = Double.MIN_VALUE;
        for (double num : nums) {
            max = Math.max(max, num);
        }
        return max;
    }

    private static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public enum ElementPosition {
        LEFT, RIGHT, NONE
    }
}
