#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/tracking/tracking.hpp"

using namespace cv;
int main(int argc, char **argv)
{
        //     VideoCapture cap;

        //     if(!cap.open(0))
        //         return 0;
        //     while(cap.isOpened())
        //     {
        //           Mat frame;
        //           Mat flippedFrame;
        //           cap.read(flippedFrame);
        //           flip(flippedFrame,frame,1);
        //           if( frame.empty() ) break;
        //           imshow("ImageView", frame);
        //           if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
        //     }

        Ptr<Tracker> tracker = TrackerMOSSE::create();
        Rect2d roi;
        Mat frame;
        VideoCapture cap("Glove_Tracking_Test.MP4");

        cap.read(frame);
        roi = selectROI("tracker", frame);

        if (roi.width == 0 || roi.height == 0)
                return 0;
        tracker->init(frame, roi);
        // perform the tracking process
        printf("Start the tracking process, press ESC to quit.\n");
        for (;;)
        {
                // get frame from the video
                cap.read(frame);
                // stop the program if no more images
                if (frame.rows == 0 || frame.cols == 0)
                        break;
                // update the tracking result
                tracker->update(frame, roi);
                // draw the tracked object
                rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
                // show image with the tracked object
                namedWindow("tracker", WINDOW_AUTOSIZE);
                imshow("tracker", frame);
                //quit on ESC button
                if (waitKey(1) == 27)
                        break;
        }

        return 0;
}
