#ifndef MY_UTILS_HPP
#define MY_UTILS_HPP

#include <Eigen/Dense>
#include <Eigen/SVD>



struct SVDResult{
    Eigen::MatrixXd U;
    Eigen::MatrixXd S;
    Eigen::MatrixXd V;
};
struct RegularizedPseudoInverseResult{
    Eigen::MatrixXd Jpinv;
    Eigen::RowVectorXd p;
};
struct iCAT_task_result{
    Eigen::MatrixXd ydotbar;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd W;
};
struct CartErrorResult{
    Eigen::Matrix<double,3,1> pos_error;
    Eigen::Matrix<double,3,1> ori_error;
};

double DecreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x){
    if(x <= xmin){
        return ymax;
    }else if(x >= xmax){
        return ymin;
    }else{
        double cosarg = (x - xmin)* M_PI / (xmax - xmin);
        return (ymax - ymin) * 0.5 * (1 + cos(cosarg)) + ymin;
    }
}
double IncreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x){
        if(x <= xmin){
            return ymin;
        }else if(x >= xmax){
            return ymax;
        }else{
            double cosarg = (x - xmin)* M_PI / (xmax - xmin) + M_PI;
            return (ymax - ymin) * 0.5 * (1 + cos(cosarg)) + ymin;
        }
    }

SVDResult computeSVD( const Eigen::MatrixXd& A ){
    SVDResult result;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    result.U = svd.matrixU();
    result.S = Eigen::MatrixXd::Zero(A.rows(), A.cols());
    result.S.topLeftCorner(svd.singularValues().size(), 
                            svd.singularValues().size()) =
                            svd.singularValues().asDiagonal();
    result.V = svd.matrixV();
    return result;
}

RegularizedPseudoInverseResult RegPseudoInverse(const Eigen::MatrixXd& X,
                                            double lambda, 
                                            double threshold){
    RegularizedPseudoInverseResult result;
    SVDResult svd_result = computeSVD(X);
   
    int lengthS = svd_result.S.rows();
    int widthS = svd_result.S.cols();
    result.p = Eigen::VectorXd::Zero(std::min(lengthS, widthS));
    result.Jpinv = Eigen::MatrixXd::Zero(X.cols(), X.rows());
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(widthS, lengthS);
    for (int i = 0; i < std::min(lengthS, widthS); ++i) {
        result.p(i)=DecreasingBellShapedFunction(0.0, threshold, 0.0, lambda, svd_result.S(i,i));
        S_inv(i,i) = svd_result.S(i,i) / (svd_result.S(i,i) * svd_result.S(i,i) +  result.p(i));
    }
    result.Jpinv = svd_result.V * S_inv * svd_result.U.transpose();
    result.p = result.p;
    return result;
}

RegularizedPseudoInverseResult iCAT_pseudoInverse(const Eigen::MatrixXd& J,
                                                const Eigen::MatrixXd& A, 
                                                const Eigen::MatrixXd& Q, 
                                                double lambda, 
                                                double threshold,
                                                double weight){
    int m = J.rows();
    int n = J.cols();
    // task oriented regularization
    Eigen::MatrixXd Rtor;
    Rtor= Eigen::MatrixXd::Zero(n,n);
    Rtor = J.transpose()*(Eigen::MatrixXd::Identity(m,m)-A)*A*J;
    // control direction regularization
    Eigen::MatrixXd Rctrl;
    Rctrl= Eigen::MatrixXd::Zero(n,n);
    Rctrl = weight * (Eigen::MatrixXd::Identity(n,n)-Q).transpose() *
                     (Eigen::MatrixXd::Identity(n,n)-Q);
    // compute the pseudo-inverse with the unifying formula

    RegularizedPseudoInverseResult ans,result;
    ans.Jpinv = Eigen::MatrixXd::Zero(n,m);
    ans.p = Eigen::VectorXd::Zero(std::min(n,m));
    result.Jpinv = Eigen::MatrixXd::Zero(n,m);
    result.p = Eigen::VectorXd::Zero(std::min(n,m));
    ans = RegPseudoInverse(J.transpose()*A.transpose()*A*J + Rtor + Rctrl, lambda, threshold);
    result.Jpinv = ans.Jpinv * J.transpose() * A.transpose() * A;
    result.p = ans.p;
    return result;
}

iCAT_task_result iCAT_Task( const Eigen::MatrixXd& A,
                            const Eigen::MatrixXd& J,
                            const Eigen::MatrixXd& Qold,
                            const Eigen::MatrixXd& rhoold,
                            const Eigen::MatrixXd& xdot,
                            double lambda,
                            double threshold,
                            double weight){
    int n = J.cols();
    if (A.cols() != J.rows()){
        throw std::invalid_argument("Dimension mismatch: A.cols() must be equal to J.rows()");
    }
    RegularizedPseudoInverseResult JQpinv1, JQpinv2;
    JQpinv1 = iCAT_pseudoInverse(J*Qold, A, Qold, lambda, threshold, weight);
    JQpinv2 = iCAT_pseudoInverse(J*Qold, A, Eigen::MatrixXd::Identity(n, n), lambda, threshold, weight);

    // compute the new projection matrix
    Eigen::MatrixXd Qnew;
    Qnew = Qold*(Eigen::MatrixXd::Identity(n,n) - JQpinv2.Jpinv*J*Qold);
    // compute W to smooth out the use of Qold in lower priority tasks
    Eigen::MatrixXd W;
    W =J*Qold*JQpinv1.Jpinv;
    Eigen::MatrixXd T;
    T = Eigen::MatrixXd::Identity(n,n) - Qold*JQpinv2.Jpinv*W*J;
    // compute the new rho
    Eigen::MatrixXd rhonew;
    rhonew = T*rhoold + Qold*JQpinv2.Jpinv*W*xdot;
    // prepare the result
    iCAT_task_result result;
    result.ydotbar = rhonew;
    result.Q = Qnew;
    result.W = W;
    return result;
}
Eigen::Vector3d VersorLemma(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
    Eigen::Vector3d i1,i2;
    Eigen::Vector3d j1,j2;
    Eigen::Vector3d k1,k2;
    i1 = R1.col(0);
    j1 = R1.col(1);
    k1 = R1.col(2);
    i2 = R2.col(0);
    j2 = R2.col(1);
    k2 = R2.col(2);
    Eigen::Vector3d sigma;
    sigma= i1.cross(i2) + j1.cross(j2) + k1.cross(k2);
    Eigen::Vector3d rosintheta;
    rosintheta= 0.5 * sigma;
    double costheta,sintheta;
    sintheta= rosintheta.squaredNorm();
    costheta= 0.5 * (i1.dot(i2) + j1.dot(j2) + k1.dot(k2) -1);
    const double VERSOR_LEMMA_TH = 1e-6;
    if(sintheta > VERSOR_LEMMA_TH){
        double theta;
        theta= std::atan2(sintheta, costheta);
        return rosintheta * (theta / sintheta);
    }
    else{
        if (costheta > 0){
            return Eigen::Vector3d::Zero() ;
        }
        else{
            // theta = pi
            Eigen::Matrix3d h = R1 + R2;
            Eigen::Vector3d temp1 = h.col(0);
            Eigen::Vector3d temp2 = h.col(1);
            Eigen::Vector3d temp3 = h.col(2);
            Eigen::Vector3d maxnormvector;
            double t1,t2,t3;
            t1 = temp1.squaredNorm();
            t2 = temp2.squaredNorm();
            t3 = temp3.squaredNorm();
            if (t2 > t1){
                if( t2 > t3){
                    maxnormvector = temp2;
                }
                else{
                    maxnormvector = temp3;
                }
            }
            else if (t3> t1){
                maxnormvector = temp3;
            }
            else{
                maxnormvector = temp1;
            }
            if (maxnormvector.squaredNorm() <= 1e-12){
                return (maxnormvector*(M_PI/maxnormvector.squaredNorm()));
            }
            else{
                return Eigen::Vector3d::Zero();
            }
        }
    }

}
CartErrorResult CartError(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2){
    CartErrorResult result;
    // position error
    result.pos_error = T1.block<3,1>(0,3)  - T2.block<3,1>(0,3);
    // orientation error
    Eigen::Matrix3d R1 = T1.block<3,3>(0,0);
    Eigen::Matrix3d R2 = T2.block<3,3>(0,0);
    result.ori_error = VersorLemma(R1, R2)*(-1.0);
    return result;
}
Eigen::MatrixXd Saturate(const Eigen::MatrixXd& x, double xmax){
    int n = x.size();
    double max=0;
    Eigen::MatrixXd out;
    for (int i=0; i<n; i++){
        if (std::abs(x(i)) > max){
            max = std::abs(x(i));
        }
    }
    if (max > xmax){
        out = (x / max) * xmax;
        return out;
    }
    else{
        out = x;
        return out;
        
    }
}
#endif // MY_UTILS_HPP

