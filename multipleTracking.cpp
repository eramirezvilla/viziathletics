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

//Global Variables

int iLowH = 0;    //0
int iHighH = 104; //104

int iLowS = 0;   //0
int iHighS = 63; //63

int iLowV = 248;  //248
int iHighV = 255; //255

int iLastX = -1;
int iLastY = -1;

//Capture a temporary image from the camera
Mat imgTemp;
Mat imgHSV;
Mat imgThresholded;
Rect gloveRec;

void createBBForGlove(Mat imgOriginal)
{

    imgTemp = imgOriginal.clone();
    //Create a black image with the size as the camera output
    Mat imgLines = Mat::zeros(imgTemp.size(), CV_8UC3);

    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);                                                 //Convert the captured frame from BGR to HSV
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //morphological closing (removes small holes from the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

     imshow("Thresholded Image", imgThresholded);

    //Calculate the moments of the thresholded image
    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
    if (dArea > 10000)
    {
        //calculate the position of the object
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;

        if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
        {
            //Draw a red line from the previous point to the current point
            //line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
            gloveRec = Rect(posX - 80, posY - 50, 150, 150);
        }
        
        rectangle(imgOriginal, gloveRec, Scalar(125, 200, 0, 0), 2, 1);
        iLastX = posX;
        iLastY = posY;
    }
}

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

//boolean values determine where the glove is in relation to the head
bool isHome = false;
bool isReturning = false;

//points for where the glove is moving
vector<Point> glovePath;

vector<Point> perfectPathPoints;

//glove points being tested against the perfect path
Point firstSectionGloveStart;
Point firstSectionGloveEnd;
Point secondSectionGloveEnd;

//points to test against the glove's location
Point startingPerfectPoint;
Point endingPerfectPoint;
Point middlePerfectPoint = Point((startingPerfectPoint.x + endingPerfectPoint.x) / 2, (startingPerfectPoint.y + endingPerfectPoint.y) / 2);

//returns the distance between point a and b using the distance formula
double distanceBetweenPoints(Point a, Point b)
{

    double xValsSquared = pow((a.x - b.x), 2);
    double yValsSquared = pow((a.y - b.y), 2);
    double totalSum = xValsSquared + yValsSquared;
    double finalResult = sqrt(totalSum);
    return finalResult;
}

//boolean values to determine which section is incorrect
//punchEvaluation is determined depending on the other 3 values
bool firstPoint;
bool secondPoint;
bool thirdPoint;
bool punchEvaluation;
bool isAnUppercut = false;
bool uppercutMiddlePointEval;

//determines whether or not the punch thrown is correct compared to the perfect path
void analyzePath(int whereInPunch)
{
    //whereInPunch refers to what stage of the punch needs to be evaluated
    switch (whereInPunch)
    {
        //starting the punch
    case 1:
        if (distanceBetweenPoints(firstSectionGloveStart, startingPerfectPoint) < 55)
        {
            //std::cout << "GOOD START" << endl;
            firstPoint = true;
        }
        else
        {
            //std::cout << "BAD START" << endl;
            firstPoint = false;
            // std::cout << "firstSectionGloveStart: " << firstSectionGloveStart << endl;
            // std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
        }
        break;
        //glove is at the end of the first stage of the punch
    case 2:
        if (distanceBetweenPoints(firstSectionGloveEnd, endingPerfectPoint) < 55)
        {
            //std::cout << "GOOD ENDING" << endl;
            secondPoint = true;
        }
        break;

        //compares the final glove position
    case 3:
        double distanceInYVals = sqrt(pow((secondSectionGloveEnd.y - startingPerfectPoint.y), 2));
        if (distanceBetweenPoints(secondSectionGloveEnd, startingPerfectPoint) < 350 && distanceInYVals < 55)
        {
            //std::cout << "GOOD END" << endl;
            thirdPoint = true;
            // std::cout << "secondSectionGloveEnd: " << secondSectionGloveEnd << endl;
            // std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
        }
        else
        {
            //std::cout << "BAD END" << endl;
            thirdPoint = false;
            // std::cout << "secondSectionGloveEnd: " << secondSectionGloveEnd << endl;
            // std::cout << "startingPerfectPoint: " << startingPerfectPoint << endl;
            // std::cout << "distance in y vals: " << distanceInYVals << endl;
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

//sets what the user's furthest reach is after the first punch is thrown
int furthestReach = 480;
bool reachIsSet = false;
bool punchIsDone = false;

bool endIsFound = false;
bool pt2EndIsFound = false;

bool needStartingPoint = true;
vector<int> combinationOfPunches;
int comboIndex = 0;

//draws the glove path line at multiple stages of the punch
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
                    //std::cout << "First Section End Here" << endl;
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
                //std::cout << "Punch done" << endl;
                if (comboIndex < combinationOfPunches.size() - 1)
                {
                    comboIndex++;
                }

                if (!pt2EndIsFound)
                {
                    secondSectionGloveEnd = glovePath.at(j);
                    analyzePath(3);
                    //std::cout << "Second Section End Here" << endl;
                }
                endIsFound = false;
                // showPunchPath();
                // punchFrames.clear();
                isReturning = false;
                punchIsDone = true;
                needStartingPoint = true;
                punchEvaluation = (firstPoint && secondPoint && thirdPoint);
                if (!punchEvaluation)
                {
                    std::cout << "_________ INCORRECT PUNCH ____________" << endl;
                    // std::cout << "First Point: " << firstPoint << endl;
                    // std::cout << "Second Point: " << secondPoint << endl;
                    // std::cout << "Third Point: " << thirdPoint << endl;
                    if(!firstPoint){
                        std::cout << "Make sure to start each punch at head level" << endl;
                    }
                    if(!secondPoint){
                        std::cout<< "Make sure your punch ends at the correct path level" << endl;
                    }
                    if(!thirdPoint){
                        std::cout <<"Make sure to bring hand back to the starting position" << endl;
                    }

                }
                if (punchEvaluation)
                {
                    std::cout << "_________ GOOD PUNCH ____________" << endl;
                }
            }
            if (j > 1 && glovePath.at(j).x > glovePath.at(j + 1).x && distanceFromStart > 200)
            {
                //std::cout << "Punch returning" << endl;
                isReturning = true;
                // if (!reachIsSet)
                // {
                //     furthestReach = glovePath.at(j).x;
                //     reachIsSet = true;
                // }
                glovePath.clear();
                break;
            }
        }
    }
}

Point startingPoint;
vector<Point> uppercutPoints;


//  Once punch is finished the new punch path is shown since the line has to be drawn again every new frame
//  If done after each punch is done correctly, I can analyze whether or not the punch is correct before showing the next
//  punch path
//  Using the vector "combinationOfPunches" might still be useful as storage for the workout
//  Once the punch is finished properly then the next number is read from the vector

//draws the perfect path line
void drawPerfectLine(Mat currFrame, Point headPos, Point glovePos, int distanceFromStart, int type)
{
    if (!(startingPoint.x > 0))
    {
        startingPoint = Point(headPos.x + (distanceFromStart / 2), headPos.y + 130);
        //std::cout<< "Distance from start: " << distanceFromStart << endl;
    }

    //type is the kind of punch thrown
    switch (type)
    {
    case 1:
        arrowedLine(currFrame, startingPoint, Point(furthestReach, headPos.y + 70), CV_RGB(255, 45, 23), 5, 4, 0);
        perfectPathPoints.push_back(Point(furthestReach, headPos.y + 70));
        startingPerfectPoint = startingPoint;
        endingPerfectPoint = Point(furthestReach, headPos.y + 70);
        isAnUppercut = false;
        break;
        //std::cout<<"Ending Point: " << endingPerfectPoint << endl;

        //std::cout << "line printed from: " << headPos.x + distanceFromStart << " , " << glovePos.y << endl;
        //std::cout << "to : " << furthestReach << " , " << headPos.y << endl;
    case 2:
        polylines(currFrame, uppercutPoints, false, CV_RGB(0,25, 230), 5, 4, 0);
        startingPerfectPoint = uppercutPoints.at(0);
        endingPerfectPoint = uppercutPoints.at(uppercutPoints.size()-1);
        isAnUppercut = true;
        break;
        //     std::cout << "Different Punch Works" << endl;
    }
}

double fpsTotal = 0;
double fpsAvg = 0;
double numFps = 0;

vector<Point> objPoints;
bool startPunch;

//Ptr<MultiTracker> multiTracker;

int main(int argc, char **argv)
{

    //vector to hold combination of punches for a full workout

    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(2);
    combinationOfPunches.push_back(2);
    combinationOfPunches.push_back(2);
    combinationOfPunches.push_back(2);
    combinationOfPunches.push_back(2);
    combinationOfPunches.push_back(2);
    combinationOfPunches.push_back(1);
    combinationOfPunches.push_back(1);

uppercutPoints.push_back(Point( 116, 342));
uppercutPoints.push_back(Point( 134, 695));
uppercutPoints.push_back(Point( 134, 695));
uppercutPoints.push_back(Point( 134, 695));
uppercutPoints.push_back(Point( 134, 695));
uppercutPoints.push_back(Point( 414, 489));
uppercutPoints.push_back(Point( 414, 424));

    //boolean to pause video manually
    bool pause = false;

    //video to be opened
    string videoPath = "One_Glove_J_U.mp4";

    //holds all selected ROIs
    //vector<Rect> bboxes;

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
    //ONLY USED WITH "One_Glove_Jab_Test.mp4"
    Rect2d headRect = Rect2d(53, 187, 161, 184);
    createBBForGlove(frame);

    //Rect gloveRect = Rect(213, 604, 171, 158);
    // bboxes.push_back(headRect);
    // bboxes.push_back(gloveRec);

    // quit if there are no bounded boxes to track
    // if (bboxes.size() < 1)
    //     return 0;
    // else
    // {
    //     int headX = bboxes.at(0).x;
    //     int headY = bboxes.at(0).y;
    //     int headWidth = bboxes.at(0).width;
    //     int headHeight = bboxes.at(0).height;
    //     int gloveX = bboxes.at(1).x;
    //     int gloveY = bboxes.at(1).y;
    //     int gloveWidth = bboxes.at(1).width;
    //     int gloveHeight = bboxes.at(1).height;

    //     std::cout << "Head: " << to_string(headX) << ", " << to_string(headY) << "\n";
    //     std::cout << "Head h x w: " << to_string(headWidth) << ", " << to_string(headHeight) << "\n";
    //     std::cout << "Glove: " << to_string(gloveX) << ", " << to_string(gloveY) << "\n";
    //     std::cout << "Glove h x w: " << to_string(gloveWidth) << ", " << to_string(gloveHeight) << "\n";
    // }

    //fills vector with random colors to use
    // vector<Scalar> colors;
    // getRandomColors(colors, 1);

    // Create multitracker
    //Ptr<MultiTracker> multiTracker = cv::MultiTracker::create();
    //multiTracker = cv::MultiTracker::create();

    // Initialize multitracker
    // for (int i = 0; i < bboxes.size(); i++)
    //     multiTracker->add(createNewTracker(), frame, Rect2d(bboxes[i]));

    //vector<Point> glovePath;

    Ptr<Tracker> tracker = TrackerCSRT::create();
    if (tracker->init(frame, headRect))
    {
        std::cout << "Initializing success" << endl;
    }
    else
    {
        std::cout << "Initializing failure" << endl;
    }

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

        createBBForGlove(frame);
        //std::cout << "uppercutPoints.push_back(Point( " << gloveRec.x << ", " << gloveRec.y << "));" << endl;

        //get tick count before updating the tracker
        double timer = (double)getTickCount();

        //Update the tracking result with new frame
        //multiTracker->update(frame);
        tracker->update(frame, headRect);

        //Draw a rectangle around the tracked objects
        // for (unsigned i = 0; i < multiTracker->getObjects().size(); i++)
        // {
        //     rectangle(frame, multiTracker->getObjects()[i], colors[i], 2, 1);
        // }
        
        
        rectangle(frame, headRect, Scalar(125, 200, 0, 0), 2, 1);

        //labels each bounded box
        // for (int n = 0; n < bboxes.size(); n++)
        // {
        //     string text = "Box number: ";
        //     text += std::to_string(n);
        //     putText(frame, text, Point(multiTracker->getObjects()[n].x, multiTracker->getObjects()[n].y), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        // }

        
        int distanceApart = gloveRec.x - headRect.x;

        if (distanceApart < 120)
        {
            //std::cout << "Starting punch" << endl;
            isHome = true;
            if (needStartingPoint)
            {
                
                firstSectionGloveStart = Point(gloveRec.x, gloveRec.y);
                analyzePath(1);
                //std::cout << "First Section Start Here" << endl;
                needStartingPoint = false;
            }
        }
        else if (distanceApart > 80)
        {
            isHome = false;
            //needStartingPoint = true;
        }

        // std::cout << "X Val Head" << multiTracker->getObjects()[0].x << "\n";
        // std::cout << "X Val Glove" << multiTracker->getObjects()[1].x << "\n";

        
        drawPerfectLine(frame, Point(headRect.x, headRect.y), Point(gloveRec.x, gloveRec.y), distanceApart, combinationOfPunches.at(comboIndex));

        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);
        fpsTotal = fpsTotal + fps;
        numFps = numFps + 1;

        //putText(frame, "FPS : " + std::to_string(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);


        Point glovePoint = Point(gloveRec.x, gloveRec.y);

        glovePath.push_back(glovePoint);

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
