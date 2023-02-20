//import FTC packages and all needed libraries
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.objdetect.Objdetect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.objdetect.QRCodeDetector;
import org.opencv.imgcodecs.Imgcodecs;


import java.util.ArrayList;
import java.util.List;

//hi

@TeleOp//set mode to TeleOp (driver control)\
@Disabled
public class WebcamExample extends LinearOpMode
{
    //hardware definitions
    OpenCvWebcam webcam;

    double minContourArea = 250.0;//minimum area that a contour is counted as a "cone" and not useless
    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        //initialize webcams
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "intakeCam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */

        webcam.setPipeline(new SamplePipeline());//tell the camera which image processing pipeline to send images to

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//start getting frames from the camera
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

           /* enum SkystoneLocation {
                LEFT,
                RIGHT,
                NONE
            }

            private int width; // width of the image
            SkystoneLocation location;

            /**
             *
             * @param width The width of the image (check your camera)
             */
            int width = 640;//width of each camera frame
            int height = 480;//height of each camera frame

            Point textAnchor;
            Scalar green = new Scalar(0, 255, 0);//define the RGB value for green

            @Override
            public void init(Mat mat)
            {
                textAnchor = new Point(20, 400);//define where to put the text onto each camera frame
            }

            @Override
            public Mat processFrame(Mat input) {
                // "Mat" stands for matrix, which is basically the image that the detector will process
                // the input matrix is the image coming from the camera
                // the function will return a matrix to be drawn on your phone's screen

                // The detector detects regular stones. The camera fits two stones.
                // If it finds one regular stone then the other must be the skystone.
                // If both are regular stones, it returns NONE to tell the robot to keep looking

                // Make a working copy of the input matrix in HSV
                Mat mat = new Mat();
                Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);//convert frame from RGB 2 HSV
                //**IMPORTANT** - HSV color ranges (0-180, 0-255, 0-255)!!!
                //for the HUE value, divide the 0-260 range by 2!!!
                Scalar lowHSV = new Scalar(150, 5, 5); // lower bound HSV for red
                Scalar highHSV = new Scalar(180, 255, 255); // higher bound HSV for red
                Mat thresh = new Mat();

                // We'll get a black and white image. The white regions represent the regular stones.
                // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
                //WE MADE A BLACK/WHITE MASK
                Core.inRange(mat, lowHSV, highHSV, thresh);


                // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
                // Oftentimes the edges are disconnected. findContours connects these edges.
                // We then find the bounding rectangles of those contours

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);//find the contours from the MASK
                //telemetry.addData("contours", contours);
                //telemetry.update();
                int largestContourArea = 0;
                int largestContour = 0;//largestContour is the INDEX. use contours.get(largestContour) to get the contour
                //find the largest contour, which we assume is the cone we're looking for
                if (contours.size() > 0) {
                    for (int i = 0; i < contours.size(); i++) {
                        if (Imgproc.contourArea(contours.get(i)) > largestContourArea && Imgproc.contourArea(contours.get(i)) > minContourArea) {
                            largestContourArea = (int) Imgproc.contourArea(contours.get(i));
                            largestContour = i;
                        }
                    }
                    if (Imgproc.contourArea(contours.get(largestContour)) > minContourArea) {//check if the largest contour is bigger than the minimum area, so we don't find dumb contours
                        //use the largest contour's moments to find the contour center
                        Moments moments = new Moments();
                        moments = (Imgproc.moments(contours.get(largestContour)));
                        int centerRow;
                        centerRow = (int) Math.round((double) (moments.get_m01() / moments.get_m00()));

                        int centerColumn;
                        centerColumn = (int) Math.round((double) (moments.get_m10() / moments.get_m00()));
                        //if a good contour is found, draw the contour onto the initial image, add a text line saying which camera we're looking at, and the contour's area
                        Imgproc.drawContours(input, contours, largestContour, new Scalar(0, 0, 255), 5);//draw the contour in the opposite color (blue cone --> red, red cone --> blue)
                        Imgproc.putText(input, String.format("Camera: intake cam. Area: %d", largestContourArea), textAnchor, Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);//print which camera it is and the contour area
                        //Mat finish = new Mat();
                        //telemetry.addData("center row: ", centerRow);
                        //telemetry.addData("center Column: ", centerColumn);
                        //telemetry.update();
                        Imgproc.circle(input, new Point(centerColumn, centerRow), 10, new Scalar(255, 255, 0), -1);//draw yellow dot at contour center
                    } else {
                        //if no good contour is found, just add a text line saying which camera we're looking at
                        Imgproc.putText(input, String.format("Camera: intake cam."), textAnchor, Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);//print which camera it is if there is no contour
                    }
                }
                else {
                        //if no good contour is found, just add a text line saying which camera we're looking at
                        Imgproc.putText(input, String.format("Camera: intake cam."), textAnchor, Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);//print which camera it is if there is no contour
                    }
                Mat qrMat = new Mat();
                Imgproc.cvtColor(input, qrMat, Imgproc.COLOR_RGB2BGR);//convert input image to BGR for QR Code detection
                QRCodeDetector qr = new QRCodeDetector();

                Mat points = new Mat();
                String data = qr.detectAndDecodeCurved(qrMat, points);

                for (int i = 0; i < points.cols(); i++) {
                    Point pt1 = new Point(points.get(0, i));
                    Point pt2 = new Point(points.get(0, (i + 1) % 4));
                    Imgproc.line(input, pt1, pt2, new Scalar(0, 255, 0), 3);
                }
                //if (!points.empty()) {
                Imgproc.putText(input, data, new Point(20, 440), Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);

                //MUST RELEASE ALL THE MAT'S WE CREATED, TO NOT LEAK MEMORY
                qrMat.release();
                points.release();
                thresh.release();
                hierarchy.release();
                mat.release();
                //return the initial image, with the text and contours added
                return input;
            }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}