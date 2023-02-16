#include "tskf.h"

TSKF::TSKF() {

    //Initialization vectors and matrices
    _x_hat << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      
    _x_tilde << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    _gamma << 0.0, 0.0, 0.0, 0.0;

    _res = Eigen::Matrix<double,6,1>::Zero();

    _V_kk_kk = Eigen::Matrix<double,12,4>::Zero();
    _P_gamma_kk_kk = Eigen::Matrix<double,4,4>::Zero();
    _P_x_kk_kk = Eigen::Matrix<double,12,12>::Zero();

    _vec << 0.0, 0.0, 0.0, 0.0, 0.0, -9.81*_Ts, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;

    _C_k << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
   
        
}

void TSKF::estimation(Eigen::Vector4d u, Eigen::MatrixXd y, Eigen::Vector4d pr_gamma) {
    //Variabili del tskf usate solo in estimation
    Eigen::Matrix<double,12,4> W_k;             //coupling
    Eigen::Matrix<double,4,4> P_gamma_kk_k;     //predizione P_gamma
    Eigen::Vector4d gamma_pred;                 //predizione gamma
    Eigen::Matrix<double,12,4> V_kk_k;          //coupling
    Eigen::Matrix<double,6,4> H_kk_k;           //coupling
    Eigen::Matrix<double,12,12> P_x_kk_k;       //predizione P_x
    Eigen::Matrix<double,12,1> x_kk_k;          //predizione x
    Eigen::Matrix<double,12,6> Kx_kk;           //guadagno kf per lo stato
    Eigen::Matrix<double,6,6> S_kk;             //residuo
    Eigen::Matrix<double,4,6> K_gamma_kk;       //guadagno kf per gamma
    Eigen::Matrix<double,4,4> U;                //input di controllo (matrice)
    Eigen::Matrix<double,4,4> I_4;              //matrice identità
    Eigen::Matrix<double,12,12> I_12;           //matrice identità

    I_4.diagonal() << 1.0, 1.0, 1.0, 1.0;
    I_12.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
     1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    
    U << u[0], 0.0, 0.0, 0.0,
    0.0, u[1], 0.0, 0.0,
    0.0, 0.0, u[2], 0.0,
    0.0, 0.0, 0.0, u[3];

    W_k = _A_k*_V_kk_kk - _B_k*U; //coupling equation (26)
    P_gamma_kk_k = _P_gamma_kk_kk + _Qgamma; //optimal bias estimator (15)
    gamma_pred = _gamma; //OBE (14)
    V_kk_k = W_k*_P_gamma_kk_kk*P_gamma_kk_k.inverse(); //coupling (27)
    H_kk_k = _C_k*V_kk_k; //coupling (28)
    P_x_kk_k = _A_k*_P_x_kk_kk*_A_k.transpose() + _Qx + W_k*_P_gamma_kk_kk*W_k.transpose()
    - V_kk_k*P_gamma_kk_k*V_kk_k.transpose(); //BFE (20)
    x_kk_k = _A_k*_x_tilde + W_k*_gamma - V_kk_k*_gamma + _vec; //BFE (19)
    _res = y - _C_k*x_kk_k; //calcolo residui (24)
    Kx_kk = P_x_kk_k*_C_k.transpose()*(_C_k*P_x_kk_k*_C_k.transpose() + _R).inverse(); //BFE (22)
    S_kk = _C_k*P_x_kk_k*_C_k.transpose() + _R; //calcolo residui (25)
    K_gamma_kk = P_gamma_kk_k*(H_kk_k.transpose())*(H_kk_k*P_gamma_kk_k*H_kk_k.transpose() + S_kk).inverse(); //OBE (17)

    //output stimatore
    _V_kk_kk = V_kk_k - Kx_kk*H_kk_k; //coupling (29)
    _P_gamma_kk_kk = (I_4 - K_gamma_kk*H_kk_k)*P_gamma_kk_k; //OBE (18)
    _P_x_kk_kk = (I_12 - Kx_kk*_C_k)*P_x_kk_k; //BFE (23)
    _x_tilde = x_kk_k + Kx_kk*( y - _C_k*x_kk_k ); //BFE (21)
    _gamma = gamma_pred + K_gamma_kk*( _res - H_kk_k*pr_gamma); //OBE (16)


}