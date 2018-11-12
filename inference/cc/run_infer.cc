// 2018, Patrick Wieschollek <mail@patwie.com>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/public/session_options.h>
#include "tensorflow/cc/ops/image_ops.h"
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
  tensorflow::MetaGraphDef graph_def;
  status = ReadBinaryProto(tensorflow::Env::Default(), graph_fn, &graph_def);
  if (status != tensorflow::Status::OK()) return status;

  // create the graph in the current session
  status = sess->Create(graph_def.graph_def());
  if (status != tensorflow::Status::OK()) return status;

  // restore model from checkpoint, iff checkpoint is given
  if (checkpoint_fn != "") {
    const std::string restore_op_name = graph_def.saver_def().restore_op_name();
    const std::string filename_tensor_name =
        graph_def.saver_def().filename_tensor_name();

    tensorflow::Tensor filename_tensor(tensorflow::DT_STRING,
                                       tensorflow::TensorShape());
    filename_tensor.scalar<std::string>()() = checkpoint_fn;

    tensor_dict feed_dict = {{filename_tensor_name, filename_tensor}};
    status = sess->Run(feed_dict, {}, {restore_op_name}, nullptr);
    if (status != tensorflow::Status::OK()) return status;
  } else {
    // virtual Status Run(const std::vector<std::pair<string, Tensor> >& inputs,
    //                  const std::vector<string>& output_tensor_names,
    //                  const std::vector<string>& target_node_names,
    //                  std::vector<Tensor>* outputs) = 0;
    status = sess->Run({}, {}, {"init"}, nullptr);
    if (status != tensorflow::Status::OK()) return status;
  }

  return tensorflow::Status::OK();
}

int main(int argc, char const *argv[]) {
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

  // same as in python file
  //auto data_ = data.flat<float>().data();
  //for (int i = 0; i < 2; ++i) data_[i] = 1;

  // read image in OpenCV
  std::string fn = "Grace_Hopper.png";
  cv::Mat image;
  image = cv::imread(fn);
  if (!image.data ) {
    std::cerr <<  "Could not open or find the image" << std::endl ;
    return -1;
  }
  // Create a 4D blob from a frame.
  Size inpSize(224, 224);
  cv::resize(image, image, inpSize)
  // convert byte to float image
  cv::Mat image_float;
  image.convertTo(image_float, CV_32FC3);
  float *image_float_data = (float*)image_float.data;

  // copy data from OpenCv to TensorFlow Tensor
  std::copy_n((char*) image_float_data, image_shape.num_elements() * sizeof(float),
                  const_cast<char*>(image_tensor.tensor_data().data()));
  /*
    upsample_size = [int(self.target_size[1] / 8 * upsample_size), int(self.target_size[0] / 8 * upsample_size)]
    self.upsample_size = self.graph.get_tensor_by_name('output_tensors/upsample_size:0')
    self.tensor_heatMat_up = self.graph.get_tensor_by_name('output_tensors/upsample_heatmat:0')
    self.tensor_pafMat_up = self.graph.get_tensor_by_name('output_tensors/upsample_pafmat:0')
    self.tensor_peaks = self.graph.get_tensor_by_name('output_tensors/tensor_peaks:0')
  */
  tensorflow::TensorShape shape({2});
  tensorflow::Tensor upsample_tensor(tensorflow::DT_FLOAT, shape);
  // same as in python file
  auto data_ = upsample_tensor.flat<float>().data();
  for (int i = 0; i < 2; ++i) data_[i] = int(224/8.0*4);
  tensor_dict feed_dict = {
      {"image:0", image_tensor}, 
      {"output_tensors/upsample_size:0",upsample_tensor},
  };
  std::vector<tensorflow::Tensor> outputs;
  TF_CHECK_OK(sess->Run(feed_dict, 
                        {"output_tensors/upsample_heatmat:0", 
                        "output_tensors/upsample_pafmat:0", 
                        "output_tensors/tensor_peaks:0"},
                        {}, &outputs));
  auto upsample_heatmat = outputs[0].tensor();//Eigen::tensor
  auto upsample_pafmat  = outputs[1].tensor();//Eigen::tensor
  auto tensor_peaks     = outputs[2].tensor();//Eigen::tensor

  std::cout << "input                 " << image_tensor.DebugString() << std::endl;
  std::cout << "upsample_heatmat" << outputs[0].DebugString() << std::endl;
  std::cout << "upsample_pafmat" << outputs[1].DebugString() << std::endl;
  std::cout << "tensor_peaks" << outputs[2].DebugString() << std::endl;
 
  return 0;
}