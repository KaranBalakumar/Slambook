#include<iostream>
#include<chrono>
using namespace std;
#include<opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/highgui/highgui.hpp>


int main(int argc, char **argv){
    //Read the image from argv[]
    cv::Mat image;
    image = cv::imread(argv[1]);
    // check wethe data is correctly loaded
    if(image.data == nullptr){
        cerr << "file " << argv[1] << "does not exist." << endl;
        return 0;
    }
    // print some basic information
    cout << "Image cols: " << image.cols << ", rows: " <<image.rows << ", channels: " 
    << image.channels() << endl;
    cv::imshow("image", image); // using cm::imshow to show the image
    cv::waitKey(0);// display and wait for a keyboard input to close
    
    // check image type
    if(image.type()!= CV_8UC1 && image.type()!=CV_8UC3){
        // we need grayscale image or rgb image
        cout << "image type incorrect." << endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y=0; y<image.rows; y++){
        unsigned char *row_ptr = image.ptr<unsigned char>(y); // row ptr is the ptr to the y-th row
        for(size_t x=0; x<image.cols; x++){
            //read the pixel on (x,y), x=column, y=row
            unsigned char *data_ptr = &row_ptr[x*image.channels()]; // data_ptr is the pointer to (x,y)
            // visit the pixel in each channel
            for(int c=0; c!=image.channels(); c++){
                unsigned char data = data_ptr[c]; // data should be poxel of I(x,y) in c-th channel
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    const auto time_used = chrono::duration_cast<chrono::microseconds>(t2 - t1);
    cout << "time used: " << time_used.count() << " microseconds." << endl;

    // operator '=' will not copy the image data but only the reference
    cv::Mat image_another = image;
    // changing image_another will also change image
    image_another(cv::Rect(0,0,100,100)).setTo(0);// set top-left 100*100 block to zero
    cv::imshow("image", image);
    cv::waitKey(0);

    // use cv::Mat::clone to actually clone the data
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("image clone", image_clone);
    cv::waitKey(0);
    
    // We are not going to copy th OpenCV's documentation here
    // do take a look into other image operations like clipping, rotation and scaling

    cv::destroyAllWindows();
    return 0;
    
}