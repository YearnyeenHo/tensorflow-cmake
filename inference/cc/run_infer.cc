#define  NUMPARTS 18
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/graph/graph.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/public/session_options.h>
#include <tensorflow/cc/ops/image_ops.h>
#include <tensorflow/cc/ops/array_ops.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

#include <vector>
#include "HumanAndBody.h"
#include "utils/pafprocess.h"
#include "utils/common.h"

typedef std::vector<std::pair<std::string, tensorflow::Tensor>> tensor_dict;

/**
 * @brief load a previous store model
 * @details [long description]
 *
 * in Python run:
 *
 *    saver = tf.train.Saver(tf.global_variables())
 *    saver.save(sess, './exported/my_model')
 *    tf.train.write_graph(sess.graph, '.', './exported/graph.pb, as_text=False)
 *
 * this relies on a graph which has an operation called `init` responsible to
 * initialize all variables, eg.
 *
 *    sess.run(tf.global_variables_initializer())  # somewhere in the python
 * file
 *
 * @param sess active tensorflow session
 * @param graph_fn path to graph file (eg. "./exported/graph.pb")
 * @param checkpoint_fn path to checkpoint file (eg. "./exported/my_model",
 * optional)
 * @return status of reloading
 */
tensorflow::Status LoadModel(tensorflow::Session *sess, std::string graph_fn,
                             std::string checkpoint_fn = "") {
  tensorflow::Status status;

  // Read in the protobuf graph we exported
  tensorflow::GraphDef graph_def;
  status = ReadBinaryProto(tensorflow::Env::Default(), graph_fn, &graph_def);
  if (status != tensorflow::Status::OK()) return status;

  // create the graph in the current session
  status = sess->Create(graph_def);
  
  return tensorflow::Status::OK();
}

void log(std::string strr)
{
  std::cout<<strr<<std::endl;
}

void draw_humans(cv::Mat& npimg, std::vector<Human>& humans)
{
  int image_w = npimg.cols;
  int image_h = npimg.rows;
  std::vector<cv::Point2f> centersVec;
  int human_id = 0;
  for(auto human : humans)
  {   
      std::vector<cv::Point2f> tmpV(NUMPARTS, cv::Point2f(-1,-1));
      centersVec = tmpV;
      // draw point
      for(int i=0; i<NUMPARTS; ++i)
      {   
        if(human.body_parts[i].part_idx == i)
        {      
          auto body_part = human.body_parts[i];
          cv::Point2f center(int(body_part.x * image_w + 0.5), int(body_part.y * image_h + 0.5));
          centersVec[i] = center;
          cv::circle(npimg, center, 3, common::CocoColors[i], 3/*thickness*/, 8/*lineType*/, 0/*shift*/);
        }
      }
      // draw line
      for(int pair_order=0; pair_order < common::CocoPairsRender.size(); ++pair_order)
      {
        cv::Point2i pair = common::CocoPairsRender[pair_order];
        if(human.body_parts[pair.x].part_idx == pair.x && human.body_parts[pair.y].part_idx == pair.y)
          cv::line(npimg, centersVec[pair.x], centersVec[pair.y], common::CocoColors[pair_order], 3);
      }
      human_id+=1;
  }
}

void tensorData2FloatArr(tensorflow::Tensor& source, std::vector<float>& fdst)
{
  std::copy_n((float*)(source.tensor_data().data()), source.shape().num_elements(),
                std::back_inserter(fdst));
}
/*ok*/
void get_shapes(tensorflow::Tensor& tens, int& d1, int& d2, int& d3)
{
  auto eten = tens.tensor<float,4>();
  d1 = eten.dimension(1);
  d2 = eten.dimension(2);
  d3 = eten.dimension(3);
}
/*ok*/
int process_paf_wrap(int p1, int p2, int p3, tensorflow::Tensor& peaks, int h1, int h2, int h3, tensorflow::Tensor& heat_mat, int f1, int f2, int f3, tensorflow::Tensor& paf_mat)
{
  // copy data from Tensorflow Tensor to float*
  std::vector<float> vPeaks, vHeatmap, vPafMat;
  //log("before tensorData2FloatArr");
  tensorData2FloatArr(peaks, vPeaks);
  tensorData2FloatArr(heat_mat, vHeatmap);
  tensorData2FloatArr(paf_mat, vPafMat);
  //log("tensorData2FloatArr OK");
  //int process_paf(int p1, int p2, int p3, std::vector<float>& peaks, int h1, int h2, int h3, std::vector<float>& heatmap, int f1, int f2, int f3, std::vector<float>& pafmap)
  process_paf(p1,p2,p3,vPeaks, h1,h2,h3,vHeatmap, f1,f2,f3,vPafMat);
  //log("process_paf OK");
  int num_humans = get_num_humans();
  //log("get_num_humans OK");
  return num_humans;
}
/*ok*/
void estimate_paf(std::vector<Human>& humans_vec, tensorflow::Tensor& peaks, tensorflow::Tensor& heat_mat, tensorflow::Tensor& paf_mat)
{
    int p1,p2,p3,h1,h2,h3,f1,f2,f3;
    //log("before get_shapes");
    get_shapes(peaks,p1,p2,p3);
    get_shapes(heat_mat,h1,h2,h3);
    get_shapes(paf_mat,f1,f2,f3);
    //log("get_shapes OK");
    int num_humans = process_paf_wrap(p1,p2,p3,peaks, h1,h2,h3,heat_mat, f1,f2,f3,paf_mat);
    //log("process_paf_wrap OK");
    for(int human_id=0; human_id<num_humans; ++human_id)
    {
      Human hman;
      bool is_added = false;
      for(int part_idx=0; part_idx<NUMPARTS; ++part_idx)
      {
        int c_idx = int(get_part_cid(human_id, part_idx));
        if (c_idx < 0)
            continue;

        is_added = true;
        hman.body_parts[part_idx] = BodyPart(
            part_idx,
            float(get_part_x(c_idx)) / h2,/*heat_mat.shape[1],*/
            float(get_part_y(c_idx)) / h1,/*heat_mat.shape[0],*/
            get_part_score(c_idx)
        );
      }
      if (is_added)
      {
        float score = get_score(human_id);
        hman.score = score;
        humans_vec.push_back(hman);
      }
    }
}
/*ok*/

int main(int argc, char const *argv[]) {
  cv::VideoCapture cap;
 // open the default camera using default API
  cap.open(0);
  const std::string graph_fn = "./expert-graph.pb";
  //const std::string checkpoint_fn = "./exported/my_model";

  // prepare session
  tensorflow::Session *sess;
  tensorflow::SessionOptions options;
  TF_CHECK_OK(tensorflow::NewSession(options, &sess));
  TF_CHECK_OK(LoadModel(sess, graph_fn));

  // prepare inputs
  tensorflow::TensorShape image_shape({1, 224,224,3});
  tensorflow::Tensor image_tensor(tensorflow::DT_FLOAT, image_shape);
  tensorflow::TensorShape shape({2});
  tensorflow::Tensor upsample_tensor(tensorflow::DT_INT32, shape);
  // same as in python file
  auto data_ = upsample_tensor.flat<int>().data();
  for (int i = 0; i < 2; ++i) data_[i] = int(224/8.0*4);

  while(true)
  {
    cv::Mat image;
    cap.read(image);
    if (!image.data ) {
      std::cerr <<  "Could not open or find the image" << std::endl ;
      return -1;
    }
    // Create a 4D blob from a frame.
    cv::Size inpSize(224, 224);
    cv::resize(image, image, inpSize);
    // convert byte to float image
    cv::Mat image_float;
    image.convertTo(image_float, CV_32FC3);
    float *image_float_data = (float*)image_float.data;

    // copy data from OpenCv to Tensorflow Tensor
    std::copy_n((char*) image_float_data, image_shape.num_elements() * sizeof(float),
                    const_cast<char*>(image_tensor.tensor_data().data()));

    tensor_dict feed_dict = {
        {"TfPoseEstimator/image:0", image_tensor}, 
        {"output_tensors/upsample_size:0",upsample_tensor},
    };

    std::vector<tensorflow::Tensor> outputs;
    auto start = std::chrono::steady_clock::now();
    auto status = sess->Run(feed_dict, 
                      {"output_tensors/upsample_heatmat:0", 
                      "output_tensors/upsample_pafmat:0", 
                      "output_tensors/tensor_peaks:0"},
                      {}, &outputs);
    auto end = std::chrono::steady_clock::now();
     
    auto heat_mat = outputs[0];
    auto paf_mat = outputs[1];
    auto peaks = outputs[2]; 
    /*
      std::cout << "input                 " << image_tensor.DebugString() << std::endl;
      std::cout << "upsample_heatmat" << outputs[0].DebugString() << std::endl;
      std::cout << "upsample_pafmat" << outputs[1].DebugString() << std::endl;
      std::cout << "tensor_peaks" << outputs[2].DebugString() << std::endl;
    */
    auto diff = end - start;
    std::cout << std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;
    
    std::vector<Human> humans_vec;
    estimate_paf(humans_vec, peaks, heat_mat, paf_mat);
    draw_humans(image, humans_vec);
    std::cout << "num human " << humans_vec.size() << std::endl;
    cv::imshow("Live", image);
    if (cv::waitKey(5) >= 0)
      break;
  } 
  return 0;
}