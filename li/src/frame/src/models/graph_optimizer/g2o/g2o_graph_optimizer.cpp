/*
Description: 
Author     : Wang Junpeng
date       : 
*/

#include "frame/models/graph_optimizer/g2o/g2o_graph_optimizer.hpp"
#include "glog/logging.h"
#include "frame/tools/tic_toc.hpp"

namespace frame {
  G2oGraphOptimizer::G2oGraphOptimizer(const std::string &solver_type) {
    graph_ptr_.reset(new g2o::SparseOptimizer()); //稀疏优化器

    g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);
    graph_ptr_->setAlgorithm(solver); //将优化算法设置给稀疏优化器

    if(!graph_ptr_->solver()) {
      LOG(ERROR) << "G2O 优化器创建失败！";
    }
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
  }

  bool G2oGraphOptimizer::Optimize() {
    static int optimize_cnt = 0;
    if(graph_ptr_->edges().size() < 1) {
      return false;
    }

    TicToc optimize_time;
    // 放入所有顶点，选择相应的边；选择所有参与优化的边和节点
    graph_ptr_->initializeOptimization();
    //通过图的边，从一个设定为origin的顶点开始计算各个其他顶点的值
    graph_ptr_->computeInitialGuess();
    //计算激活节点的总误差
    graph_ptr_->computeActiveErrors();
    //输出相关信息
    graph_ptr_->setVerbose(false);

    double chi2 = graph_ptr_->chi2(); //考虑权重的残差
    int iterations = graph_ptr_->optimize(max_iterations_num_);

    LOG(INFO) << std::endl << "------ 完成第 " << ++optimize_cnt << " 次后端优化 -------" << std::endl
              << "顶点数：" << graph_ptr_->vertices().size() << ", 边数： " << graph_ptr_->edges().size() << std::endl
              << "迭代次数： " << iterations << "/" << max_iterations_num_ << std::endl
              << "用时：" << optimize_time.toc() << std::endl
              << "优化前后误差变化：" << chi2 << "--->" << graph_ptr_->chi2()
              << std::endl << std::endl;

    return true;
  }

  bool G2oGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) {
    optimized_pose.clear();
    int vertex_num = graph_ptr_->vertices().size();

    for(int i=0; i<vertex_num; i++) {
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(i));
      Eigen::Isometry3d pose = v->estimate();
      optimized_pose.push_back(pose.matrix().cast<float>());
    }

    return true;
  }

  int G2oGraphOptimizer::GetNodeNum() {
    return graph_ptr_->vertices().size();
  }

  void G2oGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) {
    g2o::VertexSE3 *vertex(new g2o::VertexSE3());
    vertex->setId(graph_ptr_->vertices().size());
    vertex->setEstimate(pose);
    if(need_fix) {
      vertex->setFixed(true); //固定点,不在优化器中调整
    }
    graph_ptr_->addVertex(vertex);//将节点加入优化器
  }

  void G2oGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name,
        double robust_kernel_size) {
    robust_kernel_name_ = robust_kernel_name;
    robust_kernel_size_ = robust_kernel_size;
    need_robust_kernel_ = true;
}

  void G2oGraphOptimizer::AddSe3Edge(int vertex_index1,
                                     int vertex_index2,
                                     const Eigen::Isometry3d &relative_pose,
                                     const Eigen::VectorXd noise) {
    //噪声转换为信息矩阵
    Eigen::MatrixXd information_matrix = CalculateSe3EdgeInformationMatrix(noise);
    //得到两个顶点
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index2));
    //得到边
    g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_ptr_->addEdge(edge);//将边加入优化器
    if(need_robust_kernel_) {//鲁棒核的参数来自函数SetEdgeRobustKernel      
      AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }
  }

  Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    information_matrix = CalculateDiagMatrix(noise);
    return information_matrix;
  }

  void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size) {
    if (kernel_type == "NONE") {
        return;
    }
    //来自函数G2oGraphOptimizer，产生相应类型的鲁棒核
    g2o::RobustKernel *kernel = robust_kernel_factory_->construct(kernel_type);
    if (kernel == nullptr) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);
    edge->setRobustKernel(kernel);
  }

  Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
    for (int i = 0; i < noise.rows(); i++) {
        information_matrix(i, i) /= noise(i);
    }
    return information_matrix;
  }

  void G2oGraphOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                             const Eigen::Vector3d &xyz,
                                             Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ()); //定义在g2o库中的先验边
    edge->setMeasurement(xyz);//优化库中的函数
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
  }

  void G2oGraphOptimizer::AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                                    const Eigen::Quaterniond &quat,
                                                    Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = CalculateSe3PriorQuaternionEdgeInformationMatrix(noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorQuat *edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
  }

  //TODO: 姿态观测的信息矩阵尚未添加
  Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix;
    return information_matrix;
  }
}