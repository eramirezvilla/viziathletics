#include <iostream>
#include <vector>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/tracking/tracking.hpp"

using namespace cv;
using namespace std;

//method to create multiple trackers and initialize the multi tracker
Ptr<Tracker> createNewTracker()
{
    Ptr<Tracker> tracker = TrackerCSRT::create();
    return tracker;
}

//Creates random colors for each bounded box
void getRandomColors(vector<Scalar> &colors, int numColors)
{
    RNG rng(0);
    for (int i = 0; i < numColors; i++)
        colors.push_back(Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
}

bool isHome = false;
bool isReturning = false;
vector<Point> glovePath;

/**
 * Could potentially work with the clone method added but still needs to be tested
 */

// vector<Mat> punchFrames;

// void startTrackingFrames(Mat currFrame){
//     punchFrames.push_back(currFrame.clone());
// }
// void showPunchPath(){
//     for(int n = 0; n < punchFrames.size() - 1; n++){

//         imshow("Punch Path", punchFrames.at(n));
//     }
// }
vector<Point> perfectPathPoints;

// void analyzePath()
// {
//     int index = 0;
//     int currXVal = glovePath[0].x;
//     bool matchFound = false;
//     int startHerePerf;
//     int startHereGlove;

//     while (!matchFound)
//     {
//         for (int k = 0; k < perfectPathPoints.size() - 1; k++)
//         {
//             if (perfectPathPoints[k].x == currXVal)
//             {
//                 matchFound = true;
//                 startHerePerf = k;
//                 startHereGlove = index;
//                 std::cout << "Match found" << endl;
//             }
//         }
//         if (!matchFound)
//         {
//             index++;
//             currXVal = glovePath[index].x;
//             std::cout << "No match found" << endl;
//         }
//     }
//     std::cout << "Out of while now" << endl;

//     //Finds what the shortest array is to avoid an outOfBounds error
//     int newPathSize = perfectPathPoints.size() - startHerePerf;
//     int newGloveSize = glovePath.size() - startHereGlove;
//     int rulingSize;

//     if (newPathSize > newGloveSize)
//     {
//         rulingSize = newGloveSize;
//     }
//     if (newPathSize <= newGloveSize)
//     {
//         rulingSize = newPathSize;
//     }

//     std::cout << "Perfect Path: " << perfectPathPoints.size() << " Glove Path: " << glovePath.size() << endl;
//     std::cout << "Perfect Path: " << newPathSize << " Glove Path: " << newGloveSize << endl;
//     std::cout << "Start val: " << startHereGlove << " Ruling size: " << rulingSize << endl;
//     //begin comparing
//     for (int m = startHereGlove; m < rulingSize; m++)
//     {
//         std::cout << "Begin comparing" << endl;
//         if ((glovePath[m].y - perfectPathPoints[startHerePerf].y) <= 10)
//         {
//             std::cout << "___________________________________" << endl;
//         }
//         else
//         {
//             for (int l = 0; l < 10; l++)
//                 std::cout << "WRONG" << endl;
//         }
//         startHerePerf++;
//     }
//     std::cout << "Done comparing" << endl;
// }

Point firstSectionGloveStart;
Point firstSectionGloveEnd;
Point secondSectionGloveEnd;

Point startingPerfectPoint;
Point endingPerfectPoint;
Point middlePerfectPoint = Point((startingPerfectPoint.x + endingPerfectPoint.x) / 2, (startingPerfectPoint.y + endingPerfectPoint.y) / 2);

double distanceBetweenPoints(Point a, Point b)
{

    double xValsSquared = pow((a.x - b.x), 2);
    double yValsSquared = pow((a.y - b.y), 2);
    double totalSum = xValsSquared + yValsSquared;
    double finalResult = sqrt(totalSum);
    return finalResult;
}

bool firstPoint;
bool secondPoint;
bool thirdPoint;
bool punchEvaluation;

void analyzePath(int whereInPunch)
{
    switch (whereInPunch)
    {
        //starting the punch
    case 1:
        if (distanceBetweenPoints(firstSectionGloveStart, startingPerfectPoint) < 55)
        {
            std::cout << "GOOD START" << endl;
            firstPoint = true;
        }
        else
        {
            std::cout << "BAD START" << endl;
            firstPoint = false;
            std::cout << "firstSectionGloveStart: " << firstSectionGloveStart << endl;
            std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
        }
        break;
        //ending the punch
    case 2:
        if (distanceBetweenPoints(firstSectionGloveEnd, endingPerfectPoint) < 55)
        {
            std::cout << "GOOD ENDING" << endl;
            secondPoint = true;
        }
        break;
    case 3:
        double distanceInYVals = sqrt(pow((secondSectionGloveEnd.y - startingPerfectPoint.y), 2));
        if (distanceBetweenPoints(secondSectionGloveEnd, startingPerfectPoint) < 350 && distanceInYVals < 55)
        {
            std::cout << "GOOD END" << endl;
            thirdPoint = true;
            std::cout << "secondSectionGloveEnd: " << secondSectionGloveEnd << endl;
            std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
        }
        else
        {
            std::cout << "BAD END" << endl;
            thirdPoint = false;
            std::cout << "secondSectionGloveEnd: " << secondSectionGloveEnd << endl;
            std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
            std::cout << "distance in y vals: " << distanceInYVals << endl;
        }
        break;

        //     else
        // {
        //     std::cout << "NOT GOOD ENOUGH" << endl;
        //     std::cout << "firstSectionGloveStart: " << firstSectionGloveStart << endl;
        //     std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
        //     std::cout << "firstSectionGloveEnd: " << firstSectionGloveEnd << endl;
        //     std::cout << "endingPerfectPoint: " << endingPerfectPoint << endl;
        // }
    }
}

int furthestReach = 480;
bool reachIsSet = false;
bool punchIsDone = false;

bool endIsFound = false;
bool pt2EndIsFound = false;

void checkGlovePathAndDraw(Mat imgFrame, int distanceFromStart)
{
    if (!glovePath.empty())
    {
        //Mat newFrame = createOverlayFrame(imgFrame);
        for (int j = 0; j < glovePath.size() - 1; j++)
        {
            if (isHome && !isReturning)
            {
                glovePath.clear();
                break;
            }
            if (isReturning && !isHome)
            {
                if (!endIsFound)
                {
                    firstSectionGloveEnd = glovePath.at(j);
                    analyzePath(2);
                    std::cout << "First Section End Here" << endl;
                    endIsFound = true;
                }
                line(imgFrame, glovePath.at(j), glovePath.at(j + 1), CV_RGB(0, 255, 255), 5, 4, 0);
                //std::cout << "coming back" << endl;
            }
            if (!isHome && !isReturning)
            {
                line(imgFrame, glovePath.at(j), glovePath.at(j + 1), CV_RGB(0, 0, 0), 5, 4, 0);
                // std::cout << "adding frame to array" << endl;
                // startTrackingFrames(imgFrame);
                //std::cout << "going" << endl;
                punchIsDone = false;
                pt2EndIsFound = false;
            }
            if (isReturning && isHome)
            {
                std::cout << "Punch done" << endl;
                punchEvaluation = (firstPoint && secondPoint && thirdPoint);
                if (!punchEvaluation)
                {
                    std::cout << "_________ BAD PUNCH ____________" << endl;
                    std::cout << "First Point: " << firstPoint << endl;
                    std::cout << "Second Point: " << secondPoint << endl;
                    std::cout << "Third Point: " << thirdPoint << endl;
                }
                if (punchEvaluation)
                {
                    std::cout << "_________ GOOD PUNCH ____________" << endl;
                }
                if (!pt2EndIsFound)
                {
                    secondSectionGloveEnd = glovePath.at(j);
                    analyzePath(3);
                    std::cout << "Second Section End Here" << endl;
                }
                endIsFound = false;
                // showPunchPath();
                // punchFrames.clear();
                isReturning = false;
                punchIsDone = true;
            }
            if (j > 1 && glovePath.at(j).x > glovePath.at(j + 1).x && distanceFromStart > 200)
            {
                //std::cout << "Punch returning" << endl;
                isReturning = true;
                if (!reachIsSet)
                {
                    furthestReach = glovePath.at(j).x;
                    reachIsSet = true;
                }
                glovePath.clear();
                break;
            }
        }
    }
}

Point startingPoint;
/**
 * Once punch is finished the new punch path is shown since the line has to be drawn again every new frame
 * If done after each punch is done correctly, I can analyze whether or not the punch is correct before showing the next
 * punch path
 * Using the vector "combinationOfPunches" might still be useful as storage for the workout
 * Once the punch is finished properly then the next number is read from the vector
 */

void drawPerfectLine(Mat currFrame, Point headPos, Point glovePos, int distanceFromStart)
{
    if (!(startingPoint.x > 0))
    {
        startingPoint = Point(headPos.x + (distanceFromStart / 2), glovePos.y - 70);
    }
    int type = 1;

    //type is the kind of punch thrown
    switch (type)
    {
    case 1:
        arrowedLine(currFrame, startingPoint, Point(furthestReach, headPos.y + 70), CV_RGB(255, 45, 23), 5, 4, 0);
        perfectPathPoints.push_back(Point(furthestReach, headPos.y + 70));
        startingPerfectPoint = startingPoint;
        endingPerfectPoint = Point(furthestReach, headPos.y + 70);

        //std::cout << "line printed from: " << headPos.x + distanceFromStart << " , " << glovePos.y << endl;
        //std::cout << "to : " << furthestReach << " , " << headPos.y << endl;
        // case 2:
        //     arrowedLine(currFrame, startingPoint, Point(furthestReach-200, headPos.y), CV_RGB(255,45,23), 5,4,0);
        //     std::cout << "Different Punch Works" << endl;
    }
}

double fpsTotal = 0;
double fpsAvg = 0;
double numFps = 0;

vector<Point> objPoints;
bool startPunch;

Ptr<MultiTracker> multiTracker;

double checkForStart(double headx, double y, double glovex, double b)
{

    return glovex - headx;
}

bool needStartingPoint = true;

int main(int argc, char **argv)
{

    vector<int> combinationOfPunches;
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(2);

    //boolean to pause video manually
    bool pause = false;

    //video to be opened
    string videoPath = "One_Glove_Jab_Test.mp4";

    //holds all selected ROIs
    vector<Rect> bboxes;

    //opens the video by path name
    cv::VideoCapture cap(videoPath);
    Mat frame;

    //closes program if video is not found
    if (!cap.isOpened())
    {
        cout << "Error opening video file " << videoPath << "\n";
        return -1;
    }

    //read the first frame
    cap.read(frame);

    //opens selection process, exited by pressing the escape key
    // cout << "\n==========================================================\n";
    // cout << "Select the head area first then select the glove\n";
    // cout << "OpenCV says press c to cancel objects selection process";
    // cout << "\n==========================================================\n";
    // cv::selectROIs("MultiTracker", frame, bboxes, false, false);

    //Hard coded bounding boxes for consistent accuracy
    Rect headRect = Rect(60, 410, 161, 172);
    Rect gloveRect = Rect(213, 604, 171, 158);
    bboxes.push_back(headRect);
    bboxes.push_back(gloveRect);

    // quit if there are no bounded boxes to track
    if (bboxes.size() < 1)
        return 0;
    else
    {
        int headX = bboxes.at(0).x;
        int headY = bboxes.at(0).y;
        int headWidth = bboxes.at(0).width;
        int headHeight = bboxes.at(0).height;
        int gloveX = bboxes.at(1).x;
        int gloveY = bboxes.at(1).y;
        int gloveWidth = bboxes.at(1).width;
        int gloveHeight = bboxes.at(1).height;

        // std::cout << "Head: " << to_string(headX) << ", " << to_string(headY) << "\n";
        // std::cout << "Head h x w: " << to_string(headWidth) << ", " << to_string(headHeight) << "\n";
        // std::cout << "Glove: " << to_string(gloveX) << ", " << to_string(gloveY) << "\n";
        // std::cout << "Glove h x w: " << to_string(gloveWidth) << ", " << to_string(gloveHeight) << "\n";
    }

    //fills vector with random colors to use
    vector<Scalar> colors;
    getRandomColors(colors, bboxes.size());

    // Create multitracker
    //Ptr<MultiTracker> multiTracker = cv::MultiTracker::create();
    multiTracker = cv::MultiTracker::create();

    // Initialize multitracker
    for (int i = 0; i < bboxes.size(); i++)
        multiTracker->add(createNewTracker(), frame, Rect2d(bboxes[i]));

    //vector<Point> glovePath;

    while (cap.isOpened())
    {
        // get new frame from the video
        cap.read(frame);

        //Close if the video is over
        if (frame.empty())
        {
            fpsAvg = fpsTotal / numFps;
            std::cout << "Average FPS: " << std::to_string(int(fpsAvg)) << endl;
            break;
        }
        //get tick count before updating the tracker
        double timer = (double)getTickCount();

        //Update the tracking result with new frame
        multiTracker->update(frame);

        //Draw a rectangle around the tracked objects
        for (unsigned i = 0; i < multiTracker->getObjects().size(); i++)
        {
            rectangle(frame, multiTracker->getObjects()[i], colors[i], 2, 1);
        }
        for (int n = 0; n < bboxes.size(); n++)
        {
            string text = "Box number: ";
            text += std::to_string(n);
            putText(frame, text, Point(multiTracker->getObjects()[n].x, multiTracker->getObjects()[n].y), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        }
        int distanceApart = checkForStart(multiTracker->getObjects()[0].x, multiTracker->getObjects()[0].y, multiTracker->getObjects()[1].x, multiTracker->getObjects()[1].y);

        if (distanceApart < 80)
        {
            std::cout << "Starting punch" << endl;
            isHome = true;
            if (needStartingPoint)
            {
                firstSectionGloveStart = Point(multiTracker->getObjects()[1].x, multiTracker->getObjects()[1].y);
                analyzePath(1);
                std::cout << "First Section Start Here" << endl;
                needStartingPoint = false;
            }
        }
        else if (distanceApart > 80)
        {
            isHome = false;
            needStartingPoint = true;
        }

        // std::cout << "X Val Head" << multiTracker->getObjects()[0].x << "\n";
        // std::cout << "X Val Glove" << multiTracker->getObjects()[1].x << "\n";

        drawPerfectLine(frame, Point(multiTracker->getObjects()[0].x, multiTracker->getObjects()[0].y), Point(multiTracker->getObjects()[1].x, multiTracker->getObjects()[1].y), distanceApart);

        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);
        fpsTotal = fpsTotal + fps;
        numFps = numFps + 1;

        //putText(frame, "FPS : " + std::to_string(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

        int gloveXVal = multiTracker->getObjects()[1].x;
        int gloveYVal = multiTracker->getObjects()[1].y;
        Point glovePoint = Point(gloveXVal, gloveYVal);

        //FIX THIS
        // if (!punchIsDone)
        // {
        glovePath.push_back(glovePoint);
        // }
        // if (punchIsDone)
        // {
        //     glovePath.clear();
        // }
        checkGlovePathAndDraw(frame, distanceApart);

        // Show frame
        imshow("MultiTracker", frame);

        switch (waitKey(10))
        {
        //Close on 'esc' key
        case 27:
            return 0;
        case 32:
            pause = !pause;
            if (pause == true)
            {
                std::cout << "Video has been paused" << endl;
            }
            while (pause == true)
            {
                switch (waitKey())
                {
                case 32:
                    pause = false;
                    std::cout << "Video resumed" << endl;
                    break;
                case 27:
                    return 0;
                }
            }
        }
    }

    return 0;
}
