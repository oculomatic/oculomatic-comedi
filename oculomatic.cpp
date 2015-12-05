
// OpenCV offline
#include "flycapture/FlyCapture2.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <boost/circular_buffer.hpp>
#include <random>
#include <boost/circular_buffer.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <array>
#include <cstring>

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <comedilib.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>
#include <sys/mman.h>
#include <cstdlib>
#include <cmath>
#include <netdb.h>


using namespace FlyCapture2;
using namespace cv;
using namespace std;

// Initialize
comedi_t *devx;
comedi_t *devy;
#define SAMPLE_CT 5 // about as short as you can get
#define BUF_LEN 0x8000
#define COMEDI_DEVICE_AO "/dev/comedi0"

// extra feature crap
float centerX = 0;
float centerY = 0;

int center_offset_x = 0;
int center_offset_y = 0;
int max_rngx;
int max_rngy;

Vec<float,2> offset;

// initialize positions
float xpos = 0;
float ypos = 0;

// initialize initial value of input image
float xmax;
float ymax;

float amplitude_x = 4000;
float amplitude_y = 4000;
float fudge_x = 2048;
float fudge_y = 2048;

const char *comdevice = COMEDI_DEVICE_AO;
const char *comdevice2 = COMEDI_DEVICE_AO;

int external_trigger_number = 0;

lsampl_t dataxl[SAMPLE_CT];
lsampl_t datayl[SAMPLE_CT];

struct parsed_options_hold{
  char filename[256];
  float value;
  int subdevice;
  int channel;
  int aref;
  int range;
  int physical;
  int verbose;
  int n_chan;
  int n_scan;
  float freq;
};


char *cmd_src(int src,char *buf)
{
  buf[0]=0;

  if(src&TRIG_NONE)strcat(buf,"none|");
  if(src&TRIG_NOW)strcat(buf,"now|");
  if(src&TRIG_FOLLOW)strcat(buf, "follow|");
  if(src&TRIG_TIME)strcat(buf, "time|");
  if(src&TRIG_TIMER)strcat(buf, "timer|");
  if(src&TRIG_COUNT)strcat(buf, "count|");
  if(src&TRIG_EXT)strcat(buf, "ext|");
  if(src&TRIG_INT)strcat(buf, "int|");
  if(src&TRIG_OTHER)strcat(buf, "other|");

  if(strlen(buf)==0){
    sprintf(buf,"unknown(0x%08x)",src);
  }else{
    buf[strlen(buf)-1]=0;
  }

  return buf;
}


// Comedi script for 2 channel analog output
int comedi_internal_trigger_cust(comedi_t* device, int subdevice, int channelx, int channely, lsampl_t* dataxl, lsampl_t* datayl, int range, int aref)
{

  comedi_insn insn[2];
  comedi_insnlist il;

  il.n_insns=2;
  il.insns=insn;

  memset(&insn[0], 0, sizeof(comedi_insn));
  insn[0].insn = INSN_WRITE; //INSN_INTTRIG
  insn[0].subdev = subdevice;
  insn[0].data = dataxl;
  insn[0].n = SAMPLE_CT;
  insn[0].chanspec = CR_PACK(channelx,range,aref);

  memset(&insn[1], 0, sizeof(comedi_insn));
  insn[1].insn = INSN_WRITE; //INSN_INTTRIG
  insn[1].subdev = subdevice;
  insn[1].data = datayl;
  insn[1].n = SAMPLE_CT;
  insn[1].chanspec = CR_PACK(channely,range,aref);

  return comedi_do_insnlist(device, &il);
}


void dump_cmd(FILE *out,comedi_cmd *cmd)
{
  char buf[10];

  fprintf(out,"subdevice:      %d\n",
  cmd->subdev);

  fprintf(out,"start:      %-8s %d\n",
  cmd_src(cmd->start_src,buf),
  cmd->start_arg);

  fprintf(out,"scan_begin: %-8s %d\n",
  cmd_src(cmd->scan_begin_src,buf),
  cmd->scan_begin_arg);

  fprintf(out,"convert:    %-8s %d\n",
  cmd_src(cmd->convert_src,buf),
  cmd->convert_arg);

  fprintf(out,"scan_end:   %-8s %d\n",
  cmd_src(cmd->scan_end_src,buf),
  cmd->scan_end_arg);

  fprintf(out,"stop:       %-8s %d\n",
  cmd_src(cmd->stop_src,buf),
  cmd->stop_arg);
}



// tcpclient:
// A class that creates a socket to allow communication between machines
// This allows streaming data to another machine
// KEEP THIS AROUND FOR YOUR OWN GOOD!
class tcpclient{
private:
  int status;
  struct addrinfo host_info;
  struct addrinfo *host_info_list;
  int socketfd;
  const char *msg;
  int len;
  ssize_t bytes_sent;
  ssize_t bytes_recieved;
  char incoming_data_buffer[100];


public:
  void initialize(const char* hostname, const char* port){
    // need to block out memory and set to 0s
    memset(&host_info, 0, sizeof host_info);
    std::cout << "Setting up structs..." << std::endl;
    host_info.ai_family = AF_UNSPEC;
    host_info.ai_socktype = SOCK_STREAM;
    status = getaddrinfo(hostname, port, &host_info, &host_info_list);
    if (status != 0) std::cout << "getaddrinfo error" << gai_strerror(status);

    std::cout << "Creating a socket... " << std::endl;
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    if (socketfd == -1) std::cout << "Socket error";

    std::cout << "Connecting..." << std::endl;
    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1) std::cout << "Connect error";
  }
};

// CStopWatch:
// A simple timer class with Start, Stop, and GetDuration function calls
class CStopWatch{
private:
  clock_t start;
  clock_t finish;

public:
  float GetDuration() {return (float)(finish-start) / CLOCKS_PER_SEC;}
  void Start() {start = clock();}
  void Stop() {finish = clock();}

};


// Initialize global variables: These are necessary for GUI function

int min_threshold_slider_max = 100;
int min_threshold_slider;
int min_threshold;

int max_threshold_slider_max = 300;
int max_threshold_slider;
int max_threshold;

int min_area_slider_max = 50;
int min_area_slider;
int min_area;

int max_area_slider_max = 500;
int max_area_slider;
int max_area;

int min_circularity_slider_max = 300;
int min_circularity_slider;
float min_circularity;

int min_convexity_slider_max = 300;
int min_convexity_slider;
float min_convexity;

int min_inertia_slider_max = 300;
int min_inertia_slider;
float min_inertia;

int rec_slider_max = 1;
int rec_slider;
int record_video;

int centerx_slider = 0;
int centerx = 0;

int centery_slider = 0;
int centery = 0;

bool isDrawing = false;

// currentDateTime:
// Returns the current date and time
const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%H%M%S", &tstruct);

  return buf;
}

//Scale images for debug output

// Initialize Trackbars

void min_threshold_trackbar(int,void*){
  min_threshold = (int) min_threshold_slider;
}

void max_threshold_trackbar(int,void*){
  max_threshold = (int) max_threshold_slider;
}

void min_area_trackbar(int,void*){
  min_area = (int) min_area_slider * 100;
}

void max_area_trackbar(int,void*){
  max_area = (int) max_area_slider * 100;
}

void min_circularity_trackbar(int,void*){
  min_circularity = (int) min_circularity_slider / 10;
}

void min_convexity_trackbar(int,void*){
  min_convexity = (int) min_convexity_slider / 1000;
}

void min_inertia_trackbar(int,void*){
  min_inertia = (int) min_inertia_slider / 10000;
}

void rec_trackbar(int,void*){
  record_video = (int) rec_slider;
}

void centerx_trackbar(int,void*){
  center_offset_x = xpos - cvFloor(0.5*max_rngx);
}

void centery_trackbar(int,void*){
  center_offset_y = ypos - cvFloor(0.5*max_rngy);
}

// Main function:
// This program will track the eye
int main(){
  // Setup comedi
  comedi_cmd cmdx;
  comedi_cmd cmdy;
  int err;
  int n,m, i;
  int total=0, n_chan = 0, freq = 80000;
  int subdevicex = -1;
  int subdevicey = -1;
  int verbose = 0;
  unsigned int chanlistx[2];
  unsigned int chanlisty[1];
  unsigned int maxdata_x;
  unsigned int maxdata_y;
  comedi_range *rng_x;
  comedi_range *rng_y;
  int ret;
  //struct parsed_options options;
  int fn;
  int aref = AREF_GROUND;
  int range = 0;
  int channelx = 0;
  int channely = 1;
  int buffer_length;
  subdevicex = -1;
  subdevicey = -1;

  n_chan = 2;

  devx = comedi_open(comdevice);
  devy = comedi_open(comdevice2);
  if(devx == NULL){
    fprintf(stderr, "error opening %s\n", comdevice);
    return -1;
  }

  if(devy == NULL){
    fprintf(stderr,"error opening %s\n", comdevice2);
    return -1;
  }

  if(subdevicex <0)
  subdevicex = comedi_find_subdevice_by_type(devx, COMEDI_SUBD_AO, 0);
  assert(subdevicex >= 0);

  if(subdevicey <0)
  subdevicey = comedi_find_subdevice_by_type(devy, COMEDI_SUBD_AO, 0);
  assert(subdevicey >= 0);



  maxdata_x = comedi_get_maxdata(devx, subdevicex, channelx);
  rng_x = comedi_get_range(devx, subdevicex, channelx, 0);
  max_rngx = maxdata_x;

  maxdata_y = comedi_get_maxdata(devy, subdevicey, channely);
  rng_y = comedi_get_range(devy, subdevicey, channely, 0);
  max_rngy = maxdata_y;

  // initialize timer rec
  float delay;
  CStopWatch sw;

  //buffers for heuristic filtering
  boost::circular_buffer<float> buffer_x(4);
  boost::circular_buffer<float> buffer_y(4);
  float tmp1;
  float tmp2;



  // save file
  cout << "\nChoose a file name to save to. Defaults to current date and time...\n";
  string input = "";
  string filename;
  string video_filename;
  getline(cin, input);
  if (input == ""){
    filename = currentDateTime();
    video_filename = currentDateTime();
  }
  else{
    filename = input;
    video_filename = input;
  }

  filename.append(".csv");
  const char *filen = filename.c_str();

  ofstream save_file (filen);

  // Initialize camera for setup
  FlyCapture2::Error error;
  Camera camera;
  CameraInfo camInfo;

  // Connect to the camera
  error = camera.Connect(0);
  if(error != PGRERROR_OK){
    std::cout << "failed to connect to camera..." << std::endl;
    return false;
  }

  error = camera.GetCameraInfo(&camInfo);
  if (error != PGRERROR_OK){
    std::cout << "failed to get camera info from camera" << std::endl;
    return false;
  }

  std::cout << camInfo.vendorName << " "
  << camInfo.modelName << " "
  << camInfo.serialNumber << std::endl;

  error = camera.StartCapture();
  if(error==PGRERROR_ISOCH_BANDWIDTH_EXCEEDED){
    std::cout << "bandwidth exceeded" << std::endl;
    return false;
  }
  else if (error != PGRERROR_OK){
    std::cout << "failed to start image capture" << std::endl;
    return false;
  }




  // Setup: User positions eye in FOV
  //  Wait for 'c' to be pushed to move on
  cout << "Position eye inside field of view\n";
  cout << "ROI selection is now done automagically\n";
  cout << "press 1 or 2 to mirror image\n";
  cout << "press c to continue\n";
  char kb = 0;
  namedWindow("set",WINDOW_NORMAL);


  Image tmpImage;
  Image rgbTmp;
  cv::Mat tmp;
  while(kb != 'c'){
    // Grab frame from buffer
    FlyCapture2::Error error = camera.RetrieveBuffer(&tmpImage);
    if (error != PGRERROR_OK){
      std::cout<< "capture error" << std::endl;
      return false;
    }
    if(kb == '1'){
    camera.WriteRegister( 0x1054, 0x1 );
    }
    if(kb == '2'){
    camera.WriteRegister( 0x1054, 0 );
    }

    // Convert image to OpenCV color scheme
    tmpImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbTmp);
    unsigned int rowBytes = (float)rgbTmp.GetReceivedDataSize()/(float)rgbTmp.GetRows();
    tmp = cv::Mat(rgbTmp.GetRows(),rgbTmp.GetCols(),CV_8UC3,rgbTmp.GetData(),rowBytes);

    xmax = tmp.cols;
    ymax = tmp.rows;

    imshow("set",tmp);

    kb = cvWaitKey(30);
  }

  destroyWindow("set");


  // Initialize variables for sliders


  // Record video
  record_video = 0;
  rec_slider = 0;
  rec_slider_max = 1;

  centerx_slider = 0;
  centery_slider = 0;

  // Set up window with ROI and offset
  Mat window;

  // setup windows
  namedWindow("window",WINDOW_NORMAL);
  cvNamedWindow("control",WINDOW_NORMAL);
  resizeWindow("window",600,500);
  resizeWindow("control",250,80);
  moveWindow("window",0,0);
  moveWindow("control",650,0);
  bool refresh = true;

  // Initialize video recorder
  VideoWriter vid;
  float fps = 30;
  Size S = Size((int) rgbTmp.GetCols(), (int) rgbTmp.GetRows());
  video_filename = video_filename.append("-video.avi");
  vid.open(video_filename,1196444237,fps,S,true);

  // Trackbar stuff
  min_threshold_slider_max = 50;
  min_threshold_slider = 15;
  min_threshold = 15;

  max_threshold_slider_max = 100;
  max_threshold_slider = 45;
  max_threshold = 45;

  min_area_slider_max = 100;
  min_area_slider = 14;
  min_area = 1400;

  max_area_slider_max = 500;
  max_area_slider = 290;
  max_area = 29000;

  min_circularity_slider_max = 10;
  min_circularity_slider = 1;
  min_circularity = 0.1;

  min_convexity_slider_max = 1000;
  min_convexity_slider = 870;
  min_convexity = 0.87;

  min_inertia_slider_max = 1000;
  min_inertia_slider = 100;
  min_inertia = 0.01;

  // make sliders
  createTrackbar("Min Threshold", "control", &min_threshold_slider,min_threshold_slider_max, min_threshold_trackbar);
  createTrackbar("Max Threshold", "control", &max_threshold_slider,max_threshold_slider_max, max_threshold_trackbar);
  createTrackbar("Min Area", "control", &min_area_slider,min_area_slider_max, min_area_trackbar);
  createTrackbar("Max Area", "control", &max_area_slider,max_area_slider_max, max_area_trackbar);
  createTrackbar("Min Circularity", "control", &min_circularity_slider,min_circularity_slider_max, min_circularity_trackbar);
  createTrackbar("Min Convexity", "control", &min_convexity_slider,min_convexity_slider_max, min_convexity_trackbar);
  createTrackbar("Min Inertia", "control", &min_inertia_slider,min_inertia_slider_max, min_inertia_trackbar);
  createTrackbar("Record","control",&rec_slider,rec_slider_max,rec_trackbar);
  createTrackbar("center-x","control",&centerx_slider,1,centerx_trackbar);
  createTrackbar("center-y","control",&centery_slider,1,centery_trackbar);

  char key = 0;

  // This is the main loop for the function
  while(key != 'q'){

    //start timer
    sw.Start();
    Image rawImage;
    FlyCapture2::Error error = camera.RetrieveBuffer( &rawImage );
    if (error != PGRERROR_OK ){
      std::cout << "capture error" << std::endl;
      continue;
    }

    Image rgbImage;
    Image monoImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage);
    unsigned int rowBytes = (float)rgbTmp.GetReceivedDataSize()/(float)rgbTmp.GetRows();
    // convert to OpenCV Mat
    Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(),CV_8UC3, rgbImage.GetData(),rowBytes);
    Mat image_mono = Mat(monoImage.GetRows(), monoImage.GetCols(),CV_8UC1, monoImage.GetData());

    //tracking happens here

    SimpleBlobDetector::Params params;

    params.minThreshold = min_threshold;
    params.maxThreshold = max_threshold;
    params.blobColor = 0;
    params.filterByArea = true;
    params.minArea = min_area;
    params.maxArea = max_area;
    params.filterByCircularity = true;
    params.minCircularity = min_circularity;
    params.filterByConvexity = true;
    params.minConvexity = min_convexity;
    params.filterByInertia = true;
    params.minInertiaRatio = min_inertia;

    // Storage for blobs
    std::vector<KeyPoint> keypoints;

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Detect blobs
    detector->detect( image_mono, keypoints);

    Mat m_with_keypoints;
    drawKeypoints( image_mono, keypoints, m_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    if(keypoints.size() > 0){
      circle(m_with_keypoints,keypoints[0].pt,3,Scalar(255,0,0),-1,8,0);
      xpos = ((keypoints[0].pt.x-200) / (xmax-400))*(float)max_rngx;
      ypos = ((keypoints[0].pt.y-150) / (ymax-300))*(float)max_rngy;
      xpos = xpos - center_offset_x;
      ypos = ypos - center_offset_y;
    }


    if(buffer_x.size() < 3){
      buffer_x.push_front(xpos);
      buffer_y.push_front(ypos);
    }
    else{
      buffer_x.push_front(xpos);
      buffer_y.push_front(ypos);
      // filter level 1
      if(buffer_x[2] > buffer_x[1] && buffer_x[1] < buffer_x[0]){
        tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
        tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

        if(tmp2 > tmp1){
          buffer_x[1] = buffer_x[0];
        }
        else{
          buffer_x[1] = buffer_x[2] ;
        }
        buffer_x[3] = buffer_x[2];
        buffer_x[2] = buffer_x[1];
      }
      else if(buffer_x[2] < buffer_x[1] && buffer_x[1] > buffer_x[0]){
        tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
        tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

        if(tmp2 > tmp1){
          buffer_x[1] = buffer_x[0];
        }
        else{
          buffer_x[1] = buffer_x[2] ;
        }

        buffer_x[3] = buffer_x[2];
        buffer_x[2] = buffer_x[1];
      }
      else{
        buffer_x[3] = buffer_x[2];
        buffer_x[2] = buffer_x[1];
      }
      if(buffer_y[2] > buffer_y[1] && buffer_y[1] < buffer_y[0]){
        tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
        tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

        if(tmp2 > tmp1){
          buffer_y[1] = buffer_y[0];
        }
        else{
          buffer_y[1] = buffer_y[2] ;
        }
        buffer_y[3] = buffer_y[2];
        buffer_y[2] = buffer_y[1];
      }
      else if(buffer_y[2] < buffer_y[1] && buffer_y[1] > buffer_y[0]){
        tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
        tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

        if(tmp2 > tmp1){
          buffer_y[1] = buffer_y[0];
        }
        else{
          buffer_y[1] = buffer_y[2] ;
        }

        buffer_y[3] = buffer_y[2];
        buffer_y[2] = buffer_y[1];
      }
      else{
        buffer_y[3] = buffer_y[2];
        buffer_y[2] = buffer_y[1];
      }

      //downsampling

      n = SAMPLE_CT * sizeof(sampl_t);
      for (i=0; i<SAMPLE_CT; i++){
        dataxl[i] = buffer_x[2];
        datayl[i] = buffer_y[2];
      }
      //output to console
      std::cout << "\r" << dataxl[0] << "," << datayl[0] << std::flush;
      //output to analog voltage
      ret = comedi_internal_trigger_cust(devx,subdevicex,channelx, channely,dataxl,datayl,range,aref);
      //draw image
      cv::imshow("window", m_with_keypoints);

    }

    if (ret < 0){
      comedi_perror("insn error");
    }

    // Record the video - this is slow!!
    if (record_video == 1){
      vid.write(image);
      sw.Stop();
      delay = sw.GetDuration()*1000;
      std::cout << "\r" << delay << std::flush;
    }
    sw.Stop();
    delay = sw.GetDuration()*1000;
    //std::cout << "\r" << delay << std::flush;
    displayStatusBar("control",std::to_string(1000/delay));
    key = waitKey(1);

  }

}
