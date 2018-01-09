#include <jevois/Core/Module.H>
#include <jevois/Image/RawImageOps.H>
#include <sstream>
#include <iomanip>
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

 static jevois::ParameterCategory const ParamCateg("Colored Line Detector options");

JEVOIS_DEFINE_ENUM_CLASS(AlgoEnum, (Sensors) (Edges) );

JEVOIS_DECLARE_PARAMETER(sensor_algorithm, AlgoEnum, "Algorithm class to use",
                          AlgoEnum::Sensors, AlgoEnum_Values, ParamCateg);
JEVOIS_DECLARE_PARAMETER(x_sensor_count, int, "Number of sensors horizontally", 7, jevois::Range<int>(1, 100), ParamCateg);
JEVOIS_DECLARE_PARAMETER(y_sensor_count, int, "Number of horizontal sensors along edges", 15, jevois::Range<int>(1, 100), ParamCateg);

void get_major_axis_segment(cv::RotatedRect r, cv::Point2f * p1, cv::Point2f * p2) {
  cv::Point2f vertices[4];
  r.points(vertices);
  *p1 = (vertices[3]+vertices[0]) / 2.;
  *p2 = (vertices[1]+vertices[2]) / 2.;


}

class Line {
public:
  // uses general form a * x + b * y + c = 0
  double a,b,c;

  // see  https://en.wikipedia.org/wiki/Linear_equation / Two-point form, general form equation
  Line(cv::Point2f p1, cv::Point2f p2) {
    a = p2.y-p1.y;
    b = p1.x-p2.x;
    c = p2.x*p1.y-p1.x*p2.y;
  }

  double x(double y) {
    if(a == 0) return NAN;
    return ( -b * y - c) / a;
  }

  double y(double x) {
    if(b == 0) return NAN;
    return (-a*x-c)/b;
  }
};

class ColoredLineDetector : public jevois::Module,
  public jevois::Parameter<x_sensor_count, y_sensor_count, sensor_algorithm>
{
  public:
    //! Default base class constructor ok
    using jevois::Module::Module;

    //! Virtual destructor for safe inheritance
    virtual ~ColoredLineDetector() { }

    int frame_number = 0;

    void process_edges(const jevois::RawImage & inimg, jevois::RawImage & visual) {
      cv::Mat rgb = jevois::rawimage::convertToCvRGB(inimg);
      cv::Mat gray;
      cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
      int scale = 1;
      int delta = 0;
      int ddepth = CV_16S;

      cv::Mat grad_x, grad_y;
      cv::Mat abs_grad_x, abs_grad_y;
      cv::Mat grad;
      
      GaussianBlur( gray, gray, cv::Size(13,13), 0, 0, cv::BORDER_REFLECT);
      cv::Scharr( gray, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_REFLECT );
      //cv::Sobel( gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_REFLECT );

      // set left 10 pixels to zero, we get boundary effects there, even with borders
      cv::Mat roi = grad_x(cv::Rect(0, 0, 10, grad_x.size().height));
      roi.setTo(0);

      double min_grad, max_grad;
      cv::minMaxLoc(grad_x, &min_grad, &max_grad);
      // todo: avoid divide by zero
      cv::Mat r = grad_x / max_grad; 
      cv::Mat l = grad_x / min_grad;
      r.setTo(0, r>0.95);
      l.setTo(0, l>0.95);
      cv::Mat left, right;
      
      cv::convertScaleAbs(l, left, 1000); // 1000 forces overflow, so I don't have to threshold
      cv::convertScaleAbs(r, right, 1000);
      //cv::threshold(left, left, 250,255,cv::THRESH_BINARY);
      //cv::threshold(right, right, 250,255,cv::THRESH_BINARY);
      vector<cv::Mat> channels;
      channels.push_back(left);
      channels.push_back(right);
      channels.push_back(cv::Mat::zeros(left.size(), left.type()));
      cv::Mat merged;
      cv::merge(channels, merged);

      // try to find lines and draw in RGB

      for(auto channel:{0,1}) {

        // using contours and fitEllipse
        cv::RotatedRect best_ellipse;
        best_ellipse.size.width = 0;
        vector<vector<cv::Point> > contours;
        cv::findContours(channels[channel], contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,cv::Point(0,0));
        for(auto contour:contours) {
          if(contour.size() < 5) continue;
          cv::RotatedRect ellipse = cv::fitEllipse(contour);

          // save best ellipse
          if(ellipse.size.height > best_ellipse.size.height) {
            best_ellipse = ellipse;
          }
        }

        cv::Scalar color;
        color = cv::Scalar(255,255,25);
        cv::ellipse(merged, best_ellipse, color);
        stringstream label;
        cv::putText(merged, label.str().c_str(), best_ellipse.center,cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1 );

        // find x intercept
        if(best_ellipse.size.width > 0) {

          cv::Point2f p1,p2;
          get_major_axis_segment(best_ellipse, &p1, &p2);
          Line line(p1, p2);

          int bottom = merged.size().height;
          double x_intercept = line.x(bottom);
          cv::line(merged, p1, p2, cv::Scalar(255,255,255));
          cv::line(merged, cv::Point(x_intercept,bottom), cv::Point(x_intercept,bottom/2), cv::Scalar(0,255,255),2);
        }
      }

      //jevois::rawimage::pasteGreyToYUYV(right,visual,0,0);
      jevois::rawimage::pasteRGBtoYUYV(merged,visual,0,0);
    }

    char sense_color_at_rect(cv::Rect roi, cv::Mat & rgb, jevois::RawImage & visual) {
        cv::Mat sensor_rgb = rgb(roi);
        //cv::Mat sensor_lab;
        //cv::cvtColor(sensor_rgb, sensor_rgb, cv::COLOR_RGB2Lab);
        cv::Mat planes[3];
        cv::split(sensor_rgb,planes);
        auto median_r = median(planes[0],256);
        auto median_g = median(planes[1],256);
        auto median_b = median(planes[2],256);

        char c =  color_name(median_r, median_g, median_b);
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
            jevois_c = jevois::yuyv::DarkPurple;
          } else if (c =='k') {
            jevois_c = jevois::yuyv::Black;
          }
          const int thickness = 3;
          jevois::rawimage::writeText(visual, ss.str().c_str(), roi.x, roi.y-18, jevois::yuyv::White, jevois::rawimage::Font6x10);
          jevois::rawimage::drawRect(visual, roi.x-thickness, roi.y-thickness, roi.width+thickness*2, roi.height+thickness*2, thickness, jevois_c);
        }
      return c;
    }

    void process_virtual_sensors(const jevois::RawImage & inimg, jevois::RawImage & visual) {
      // Just copy the pixel data over:
      if(visual.valid()) {
        memcpy(visual.pixelsw<void>(), inimg.pixels<void>(), std::min(inimg.buf->length(), visual.buf->length()));
      }

      cv::Mat rgb = jevois::rawimage::convertToCvRGB(inimg);
      auto image_width = rgb.cols;
      auto image_height = rgb.rows;
      auto num_sensors_x = x_sensor_count::get();
      auto num_sensors_y = y_sensor_count::get();
      auto sensor_x_spacing = image_width/num_sensors_x;
      auto sensor_y_spacing = image_height/num_sensors_y;
      auto sensor_height = 10;
      auto sensor_width = 10;
      stringstream serial_message;
      serial_message << "cld "; // cld is the message identifier
      auto sensor_cy = image_height - 20;
      if(visual.valid()) {
        // background rect for text
        jevois::rawimage::drawFilledRect(visual, 0, sensor_cy-20, image_width ,20, jevois::yuyv::DarkGrey);
      }
      for(int i =0; i < num_sensors_x; i++) {
        // extract sub-image in sensor region
        auto sensor_cx = sensor_x_spacing * i + sensor_x_spacing/2;
        auto roi = cv::Rect(sensor_cx - sensor_width/2, sensor_cy + sensor_height / 2, sensor_width, sensor_height);
        serial_message << sense_color_at_rect(roi, rgb, visual);
      }
      
      for(int col : {0,num_sensors_x-1}) {
        serial_message << " ";
        for(int row = 0; row < num_sensors_y; row++) {
          auto sensor_cx = sensor_x_spacing * col + sensor_x_spacing/2;
          sensor_cy = image_height - (sensor_y_spacing * row + sensor_y_spacing/2);
          auto roi = cv::Rect(sensor_cx - sensor_width/2, sensor_cy - sensor_height / 2, sensor_width, sensor_height);
          serial_message << sense_color_at_rect(roi, rgb, visual);
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

    // this non-virtual form of process will be called by both virtual process methods
    // so, visual might not be valid
    void process(const jevois::RawImage & inimg, jevois::RawImage & visual) {

      if(sensor_algorithm::get() == AlgoEnum::Edges) {
        process_edges(inimg, visual);
        return;
      }
      process_virtual_sensors(inimg, visual);

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

