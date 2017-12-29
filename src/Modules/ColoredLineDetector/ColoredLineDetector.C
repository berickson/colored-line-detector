// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
// redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
// Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.  You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

#include <jevois/Core/Module.H>
//#include <jevois/Core/Serial.H>
#include <jevois/Image/RawImageOps.H>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

char color_name(float r, float g, float b){
  if (r < 0.05 && g < 0.05) {
    return 'k';
  }
  if (r > 2 * g){
    return 'r';
  }
  if (b > 2 * r && b > 2 * g){
    return 'b';
  }
  if (g > 2 * r){
    return 'g';
  }

  return 'w';
}


double median(cv::Mat Input, int nVals){

// COMPUTE HISTOGRAM OF SINGLE CHANNEL MATRIX
float range[] = { 0, float(nVals) };
const float* histRange = { range };
bool uniform = true; bool accumulate = false;
cv::Mat hist;
calcHist(&Input, 1, 0, cv::Mat(), hist, 1, &nVals, &histRange, uniform, accumulate);

// COMPUTE CUMULATIVE DISTRIBUTION FUNCTION (CDF)
cv::Mat cdf;
hist.copyTo(cdf);
for (int i = 1; i <= nVals-1; i++){
    cdf.at<float>(i) += cdf.at<float>(i - 1);
}
cdf /= Input.total();

// COMPUTE MEDIAN
double medianVal = NAN;
for (int i = 0; i <= nVals-1; i++){
    if (cdf.at<float>(i) >= 0.5) { medianVal = i;  break; }
}
return medianVal/nVals; }


// icon by Catalin Fertu in cinema at flaticon

//! JeVois sample module
/*! This module is provided as an example of how to create a new standalone module.

    JeVois provides helper scripts and files to assist you in programming new modules, following two basic formats:

    - if you wish to only create a single module that will execute a specific function, or a collection of such modules
      where there is no shared code between the modules (i.e., each module does things that do not relate to the other
      modules), use the skeleton provided by this sample module. Here, all the code for the sample module is compiled
      into a single shared object (.so) file that is loaded by the JeVois engine when the corresponding video output
      format is selected by the host computer.

    - if you are planning to write a collection of modules with some shared algorithms among several of the modules, it
      is better to first create machine vision Components that implement the algorithms that are shared among several of
      your modules. You would then compile all your components into a first shared library (.so) file, and then compile
      each module into its own shared object (.so) file that depends on and automatically loads your shared library file
      when it is selected by the host computer. The jevoisbase library and collection of components and modules is an
      example for how to achieve that, where libjevoisbase.so contains code for Saliency, ObjectRecognition, etc
      components that are used in several modules, and each module's .so file contains only the code specific to that
      module.

    @author Brian Erickson

    @videomapping YUYV 639 480 28.5 YUYV 640 480 28.5 BrianErickson ColoredLineDetector
    @email berickson\@gmail.com
    @distribution Unrestricted
    @restrictions None */
class ColoredLineDetector : public jevois::Module
{
  public:
    //! Default base class constructor ok
    using jevois::Module::Module;

    //! Virtual destructor for safe inheritance
    virtual ~ColoredLineDetector() { }

    int frame_number = 0;

    //! Processing function
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
      ++frame_number;

      // Wait for next available camera image:
      jevois::RawImage const inimg = inframe.get(true);


      // We only support YUYV pixels in this example, any resolution:
      inimg.require("input", inimg.width, inimg.height, V4L2_PIX_FMT_YUYV);

      // Wait for an image from our gadget driver into which we will put our results:
      jevois::RawImage outimg = outframe.get();
      memcpy(outimg.pixelsw<void>(), inimg.pixels<void>(), std::min(inimg.buf->length(), outimg.buf->length()));

      // Enforce that the input and output formats and image sizes match:
      outimg.require("output", inimg.width, inimg.height, inimg.fmt);
      
      // Just copy the pixel data over:
      cv::Mat rgb = jevois::rawimage::convertToCvRGB(inimg);
      auto image_width = rgb.cols;
      auto image_height = rgb.rows;
      auto num_sensors = 10;
      auto sensor_width = image_width/num_sensors;
      auto sensor_height = sensor_width;
      stringstream serial_message;
      for(int i =0; i < num_sensors; i++) {
        // extract sub-image in sensor region
        auto roi = cv::Rect(sensor_width*i,(image_height-sensor_height)/2,sensor_width,sensor_height);
        cv::Mat sensor_rgb = rgb(roi);
        cv::Mat planes[3];
        cv::split(sensor_rgb,planes);
        auto median_r = median(planes[0],256);
        auto median_g = median(planes[1],256);
        auto median_b = median(planes[2],256);
        stringstream ss;
        ss.precision(2);
        ss.width(4);
        ss.setf( std::ios::fixed, std:: ios::floatfield );
        char c =  color_name(median_r, median_g, median_b);
        serial_message << c;
        ss << c << " : " << median_r << "," << median_g << "," << median_b;
        int jevois_c = jevois::yuyv::White;
        if (c=='r') {
          jevois_c = jevois::yuyv::DarkPink;
        } else if (c == 'g') {
          jevois_c = jevois::yuyv::MedGreen;
        } else if (c == 'b') {
          jevois_c = jevois::yuyv::DarkTeal;
        } else if (c =='k') {
          jevois_c = jevois::yuyv::Black;
        }
        const int thickness = 3;
        jevois::rawimage::drawFilledRect(outimg, roi.x, roi.y-20, sensor_width,20, jevois::yuyv::DarkGrey);
        jevois::rawimage::writeText(outimg, ss.str().c_str(), roi.x, roi.y-18, jevois::yuyv::White, jevois::rawimage::Font6x10);
        jevois::rawimage::drawRect(outimg, roi.x, roi.y, sensor_width, sensor_height, thickness, jevois_c);
      }

      // Send detected colors over usb
      sendSerial(serial_message.str());

      // Print a text message:
      //jevois::rawimage::writeText(outimg, "Hello JeVois!", 50, 50, jevois::yuyv::White, jevois::rawimage::Font20x38);
      stringstream ss;
      ss << "Frame: " << frame_number;
      jevois::rawimage::writeText(outimg, ss.str().c_str(), 50, 90, jevois::yuyv::White, jevois::rawimage::Font6x10);
      
      // Let camera know we are done processing the input image:
      inframe.done(); // NOTE: optional here, inframe destructor would call it anyway

      // Send the output image with our processing results to the host over USB:
      outframe.send(); // NOTE: optional here, outframe destructor would call it anyway
    }
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(ColoredLineDetector);
