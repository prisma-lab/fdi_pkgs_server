#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/tf.h"
#include <Eigen/Geometry>
#include "../utils.h"

using namespace std;
using namespace Eigen;

class TSKF {

    public:

        TSKF();

        void set_matrix(double mass_, Eigen::Matrix3d J_, double gravity_, double K_);
        void estimation(Eigen::Vector4d pwm, Eigen::MatrixXd y, Eigen::Vector4d pr_gamma);
        void change_inputs(Eigen::Vector4d u, Eigen::Vector3d p, Eigen::Vector3d w);

    private:

        //Estimator Inputs
        Eigen::Vector4d _u_k;
        Eigen::Matrix<double,6,1> _y_kk;
        
        //Estimator Outputs
        Eigen::Matrix<double,12,1>  _x_hat;
        //in the loop
        Eigen::Matrix<double,12,1> _x_tilde;
        Eigen::Vector4d _gamma;

        //Matrices used in TSKF

        //in the loop
        Eigen::Matrix<double,12,4> _V_kk_kk;
        Eigen::Matrix<double,4,4> _P_gamma_kk_kk;
        Eigen::Matrix<double,12,12> _P_x_kk_kk;
        Eigen::Matrix<double,6,1> _res;

        //out of the loop (vanno inizializzati nello stimatore)
        // Eigen::Matrix<double,12,4> W_k;
        // Eigen::Matrix<double,4,4> P_gamma_kk_k;
        // Eigen::Vector4d

        Eigen::Matrix<double,12,1> _vec;

        //Matrices modello linearizzato
        Eigen::Matrix<double,12,12> _A_k;
        Eigen::Matrix<double,12,4> _B_k;
        Eigen::Matrix<double,6,12> _C_k;

        //Matrici probabilistiche
        Eigen::Matrix<double,12,12> _Qx;
        Eigen::Matrix<double,4,4> _Qgamma;
        Eigen::Matrix<double,6,6> _R;

        double _Ts = 0.01;



        




};