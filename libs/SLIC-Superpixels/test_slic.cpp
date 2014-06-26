/*
 * test_slic.cpp.
 *
 * Written by: Pascal Mettes.
 *
 * This file creates an over-segmentation of a provided image based on the SLIC
 * superpixel algorithm, as implemented in slic.h and slic.cpp.
 */
 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
#include <ctime>
using namespace std;
using namespace cv;

#include "slic.h"

int main(int argc, char *argv[]) {
    /* Load the image and convert to Lab colour space. */
    VideoCapture inputVideo(argv[1]);
    if(!inputVideo.isOpened())
    {
        cout << "Could not open the input video" << endl;
        return -1;
    }
    const string NAME = argv[4];   // Form the new name with container
    int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form

    // Transform from int to char via Bitwise operators
    char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};

    Size S = Size((int) inputVideo.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                  (int) inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));

    cout << "Input frame resolution: Width=" << S.width << "  Height=" << S.height
         << " of nr#: " << inputVideo.get(CV_CAP_PROP_FRAME_COUNT) << endl;
    cout << "Input codec type: " << EXT << endl;

    cout << "Output file name: " << NAME << endl;
    //VideoWriter outputVideo;
    //outputVideo.open(NAME, ex=-1, inputVideo.get(CV_CAP_PROP_FPS), S, true);
    VideoWriter outputVideo(NAME, ex=878070084, inputVideo.get(CV_CAP_PROP_FPS), S, true);
    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write" << endl;
        return -1;
    }

    Mat src, res;
    vector<Mat> spl;

    for(int i = 0;;i++) //Show the image captured in the window and repeat
    {
        inputVideo >> src;              // read
        if (src.empty()) break;         // check if at end

        cout << "Frame: " << i << endl;
		
        // awkward conversion
        IplImage image_tmp = src;
        IplImage *image = &image_tmp;

        //IplImage *image = cvLoadImage(argv[1], 1);
        IplImage *lab_image = cvCloneImage(image);
        cvCvtColor(image, lab_image, CV_BGR2Lab);
        
        // Yield the number of superpixels and weight-factors from the user.
        int w = image->width, h = image->height;
        int nr_superpixels = atoi(argv[2]);
        int nc = atoi(argv[3]);

        double step = sqrt((w * h) / (double) nr_superpixels);
        
        // Perform the SLIC superpixel algorithm. 
/*
        clock_t start, end;
        double duration;
        start = std::clock();
        cout<<"start: "<< start <<'\n';
*/
        Slic slic;
        slic.generate_superpixels(lab_image, step, nc);
        slic.create_connectivity(lab_image);
        // Display the contours and show the result.
        //slic.display_contours(image, CV_RGB(255,0,0));
        slic.colour_with_cluster_means(image);
/*
        end = clock();
        cout<<"end: "<< end <<'\n';
        duration = ( end - start ) / (double) CLOCKS_PER_SEC;
        cout<<"printf: "<< duration <<'\n';
*/
        outputVideo << image;

		cvReleaseImageHeader(&image);
    }
    cout << "Finished writing" << endl;
    return 0;

}
