/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include "opencv2/opencv.hpp"

extern "C"
{
#include "apriltag.h"
#include "tagGrid36h11.h"
}

using namespace std;
using namespace cv;

class AprilGrid
{
private:
  apriltag_family_t *tf = NULL;
  apriltag_detector_t *td = NULL;

public:
  AprilGrid(unsigned int blackTagBorder = 2)
  {
    tf = tagGrid36h11_create(blackTagBorder);
    td = apriltag_detector_create();

    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1;
    td->quad_sigma = 0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;
    td->qtp.erode = 1;
  }

  zarray_t *detectAprilGrid(const cv::Mat &image)
  {
    image_u8_t im = {.width = image.cols,
                     .height = image.rows,
                     .stride = image.cols,
                     .buf = image.data};
    clock_t a = clock();
    zarray_t *detections = apriltag_detector_detect(td, &im);

    return detections;
  }
  ~AprilGrid()
  {
    apriltag_detector_destroy(td);
    tagGrid36h11_destroy(tf);
  }
};

int main(int argc, char *argv[])
{

  if (argc != 2)
  {
    cout << "input errror\nUsage: aprilgrid_demo image_file\n";
    return -1;
  }
  char *image_file = argv[1];

  Mat frame;
  cv::Mat gray = cv::imread(image_file, 0);
  if (!gray.empty())
  {
    cv::cvtColor(gray, frame, CV_GRAY2BGR);
  }
  else
  {
    cout << "read error " << image_file << endl;
    return -1;
  }

  AprilGrid ag(2);
  zarray_t *detections = ag.detectAprilGrid(gray);

  for (int i = 0; i < zarray_size(detections); i++)
  {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    line(frame, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[1][0], det->p[1][1]),
         Scalar(0, 0xff, 0), 2);
    line(frame, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[3][0], det->p[3][1]),
         Scalar(0, 0, 0xff), 2);
    line(frame, Point(det->p[1][0], det->p[1][1]),
         Point(det->p[2][0], det->p[2][1]),
         Scalar(0xff, 0, 0), 2);
    line(frame, Point(det->p[2][0], det->p[2][1]),
         Point(det->p[3][0], det->p[3][1]),
         Scalar(0xff, 0, 0), 2);

    stringstream ss;
    ss << det->id;
    String text = ss.str();
    int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    Size textsize = getTextSize(text, fontface, fontscale, 2,
                                &baseline);
    putText(frame, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
            fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
  }
  apriltag_detections_destroy(detections);

  imshow("Tag Detections", frame);
  waitKey();

  return 0;
}
