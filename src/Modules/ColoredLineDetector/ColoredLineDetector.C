#include <jevois/Core/Module.H>
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

  return '.';
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

// ColoredLineDetection module
//
//    @author Brian Erickson
//
//    @videomapping YUYV 639 480 28.5 YUYV 640 480 28.5 BrianErickson ColoredLineDetector
//    @email berickson\@gmail.com
//    @distribution Unrestricted
//    @restrictions None */
class ColoredLineDetector : public jevois::Module
{
  public:
    //! Default base class constructor ok
    using jevois::Module::Module;

    //! Virtual destructor for safe inheritance
    virtual ~ColoredLineDetector() { }

    int frame_number = 0;

    // this non-virtual form of process will be called by both virtual process methods
    // so, visual might not be valid
    void process(const jevois::RawImage & inimg, jevois::RawImage & visual) {
      // Just copy the pixel data over:
      if(visual.valid()) {
        memcpy(visual.pixelsw<void>(), inimg.pixels<void>(), std::min(inimg.buf->length(), visual.buf->length()));
      }

    
      cv::Mat rgb = jevois::rawimage::convertToCvRGB(inimg);
      auto image_width = rgb.cols;
      auto image_height = rgb.rows;
      auto num_sensors = 7;
      auto sensor_spacing = image_width/num_sensors;
      auto sensor_height = 10;
      auto sensor_width = 10;
      stringstream serial_message;
      serial_message << "cld "; // cld is the message identifier
      auto sensor_cy = image_height - 20;
      if(visual.valid()) {
        // background rect for text
        jevois::rawimage::drawFilledRect(visual, 0, sensor_cy-20, image_width ,20, jevois::yuyv::DarkGrey);
      }
      for(int i =0; i < num_sensors; i++) {
        // extract sub-image in sensor region
        auto sensor_cx = sensor_spacing * i + sensor_spacing/2;
        auto roi = cv::Rect(sensor_cx - sensor_width/2, sensor_cy + sensor_height / 2, sensor_width, sensor_height);
        cv::Mat sensor_rgb = rgb(roi);
        //cv::Mat sensor_lab;
        //cv::cvtColor(sensor_rgb, sensor_rgb, cv::COLOR_RGB2Lab);
        cv::Mat planes[3];
        cv::split(sensor_rgb,planes);
        auto median_r = median(planes[0],256);
        auto median_g = median(planes[1],256);
        auto median_b = median(planes[2],256);

        char c =  color_name(median_r, median_g, median_b);
        serial_message << c;
        if(visual.valid()) {
          stringstream ss;
          ss << c << std::setw(2) << std::setfill('0') << int(100*median_r) 
          << std::setw(2) << std::setfill('0') << int(100*median_g) 
          << std::setw(2) << std::setfill('0')<< int(100*median_b);
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
          jevois::rawimage::writeText(visual, ss.str().c_str(), i*sensor_spacing, roi.y-18, jevois::yuyv::White, jevois::rawimage::Font6x10);
          jevois::rawimage::drawRect(visual, roi.x-thickness, roi.y-thickness, roi.width+thickness*2, roi.height+thickness*2, thickness, jevois_c);
        }
      }
      if(visual.valid()) {
        // Print a text message:
        stringstream ss;
        ss << "Frame: " << frame_number;
        jevois::rawimage::writeText(visual, ss.str().c_str(), 50, 90, jevois::yuyv::White, jevois::rawimage::Font6x10);
      }
      // Send detected colors over usb
      sendSerial(serial_message.str());

    }

    virtual void process(jevois::InputFrame && frame) {
      jevois::RawImage visual; // unallocated pixels, will not draw anything
      jevois::RawImage img = frame.get(true);
      process(img, visual);
    }

    //! Processing function
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
      ++frame_number;

      // Wait for next available camera image:
      jevois::RawImage const inimg = inframe.get(true);


      // We only support YUYV pixels in this example, any resolution:
      inimg.require("input", inimg.width, inimg.height, V4L2_PIX_FMT_YUYV);

      // Wait for an image from our gadget driver into which we will put our results:
      jevois::RawImage visual = outframe.get();

      // Enforce that the input and output formats and image sizes match:
      visual.require("output", inimg.width, inimg.height, inimg.fmt);

      process(inimg, visual);
      
      // Let camera know we are done processing the input image:
      inframe.done(); // NOTE: optional here, inframe destructor would call it anyway

      // Send the output image with our processing results to the host over USB:
      outframe.send(); // NOTE: optional here, outframe destructor would call it anyway
    }
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(ColoredLineDetector);
