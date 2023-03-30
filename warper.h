
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>




double rad2Deg(double rad){return rad*(180/M_PI);}//Convert radians to degrees
double deg2Rad(double deg){return deg*(M_PI/180);}//Convert degrees to radians


class Warper
{


public:

void warpImage(const cv::Mat &src,
            cv::Mat&      dst)
{
    double halfFovy=m_fovy*0.5;
    double d=hypot(src.cols,src.rows);
    double sideLengthX=m_scaleX*d/cos(deg2Rad(halfFovy));

    double sideLengthY=m_scaleY*d/cos(deg2Rad(halfFovy));

    cv::Mat      M;
    std::vector<cv::Point2f> corners;

    warpMatrix(src.size(),m_theta,m_phi,m_gamma, m_scaleX,m_scaleY, m_fovy,m_x,m_y,m_z,M,&corners);//Compute warp matrix
    warpPerspective(src,dst,M,cv::Size(sideLengthX,sideLengthY));//Do actual image warp
}

//
// Theta
//

double GetTheta()
{
    return m_theta;
}

void SetTheta(double Value)
{
    m_theta = Value;
}

//
// Phi
//


double GetPhi()
{
    return m_phi;
}

void SetPhi(double Value)
{
    m_phi = Value;
}

//
// Gamma
//

double GetGamma()
{
    return m_gamma;
}

void SetGamma(double Value)
{
    m_gamma = Value;
}

//
// Scale X
//

double GetScaleX()
{
    return m_scaleX;
}

void SetScaleX(double Value)
{
    m_scaleX = Value;
}


//
// Scale Y
//

double GetScaleY()
{
    return m_scaleY;
}

void SetScaleY(double Value)
{
    m_scaleY = Value;
}


//
// FOVY
//


double GetFovY()
{
    return m_fovy;
}

void SetFovY(double Value)
{
    m_fovy = Value;
}


//
// X Offset
//

double GetXOffset()
{
    return m_x;
}

void SetXOffset(double Value)
{
    m_x = Value;
}

//
// Y Offset
//

double GetYOffset()
{
    return m_y;
}

void SetYOffset(double Value)
{
    m_y = Value;
}


//
// Z Offset
//

double GetZOffset()
{
    return m_z;
}

void SetZOffset(double Value)
{
    m_z = Value;
}


private:
    double m_theta = 5;
    double m_phi = 50;
    double m_gamma = 0;
    double m_scaleX = 1;
    double m_scaleY = 1;
    double m_fovy = 30;
    double m_x = 0;
    double m_y = 0;
    double m_z = 0;




    void warpMatrix(cv::Size   sz,
                double theta,
                double phi,
                double gamma,
                double scaleX,
                double scaleY,
                double fovy,
                double x,
                double y,
                double z,
                cv::Mat&   M,
                std::vector<cv::Point2f>* corners){
    double st=sin(deg2Rad(theta));
    double ct=cos(deg2Rad(theta));
    double sp=sin(deg2Rad(phi));
    double cp=cos(deg2Rad(phi));
    double sg=sin(deg2Rad(gamma));
    double cg=cos(deg2Rad(gamma));

    double halfFovy=fovy*0.5;
    double d=hypot(sz.width,sz.height);
    double sideLengthX=scaleX*d/cos(deg2Rad(halfFovy));
    double sideLengthY=scaleY*d/cos(deg2Rad(halfFovy));
    double h=d/(2.0*sin(deg2Rad(halfFovy)));
    double n=h-(d/2.0);
    double f=h+(d/2.0);

    cv::Mat F=cv::Mat(4,4,CV_64FC1);//Allocate 4x4 transformation matrix F
    cv::Mat Rtheta=cv::Mat::eye(4,4,CV_64FC1);//Allocate 4x4 rotation matrix around Z-axis by theta degrees
    cv::Mat Rphi=cv::Mat::eye(4,4,CV_64FC1);//Allocate 4x4 rotation matrix around X-axis by phi degrees
    cv::Mat Rgamma=cv::Mat::eye(4,4,CV_64FC1);//Allocate 4x4 rotation matrix around Y-axis by gamma degrees

    cv::Mat T=cv::Mat::eye(4,4,CV_64FC1);//Allocate 4x4 translation matrix along Z-axis by -h units
    cv::Mat P=cv::Mat::zeros(4,4,CV_64FC1);//Allocate 4x4 projection matrix

    //Rtheta
    Rtheta.at<double>(0,0)=Rtheta.at<double>(1,1)=ct;
    Rtheta.at<double>(0,1)=-st;Rtheta.at<double>(1,0)=st;
    //Rphi
    Rphi.at<double>(1,1)=Rphi.at<double>(2,2)=cp;
    Rphi.at<double>(1,2)=-sp;Rphi.at<double>(2,1)=sp;
    //Rgamma
    Rgamma.at<double>(0,0)=Rgamma.at<double>(2,2)=cg;
    Rgamma.at<double>(0,2)=-sg;Rgamma.at<double>(2,0)=sg;

    //T
    T.at<double>(2,3)=-h + z;
    T.at<double>(1,3)= y;
    T.at<double>(0,3)= x;


    //P
    P.at<double>(0,0)=P.at<double>(1,1)=1.0/tan(deg2Rad(halfFovy));
    P.at<double>(2,2)=-(f+n)/(f-n);
    P.at<double>(2,3)=-(2.0*f*n)/(f-n);
    P.at<double>(3,2)=-1.0;


    //Compose transformations
    F=P*T*Rphi*Rtheta*Rgamma;//Matrix-multiply to produce master matrix

    //Transform 4x4 points
    double ptsIn [4*3];
    double ptsOut[4*3];
    double halfW=sz.width/2, halfH=sz.height/2;

    ptsIn[0]=-halfW;ptsIn[ 1]= halfH;
    ptsIn[3]= halfW;ptsIn[ 4]= halfH;
    ptsIn[6]= halfW;ptsIn[ 7]=-halfH;
    ptsIn[9]=-halfW;ptsIn[10]=-halfH;
    ptsIn[2]=ptsIn[5]=ptsIn[8]=ptsIn[11]=0;//Set Z component to zero for all 4 components

    cv::Mat ptsInMat(1,4,CV_64FC3,ptsIn);
    cv::Mat ptsOutMat(1,4,CV_64FC3,ptsOut);

    perspectiveTransform(ptsInMat,ptsOutMat,F);//Transform points

    //Get 3x3 transform and warp image
    cv::Point2f ptsInPt2f[4];
    cv::Point2f ptsOutPt2f[4];

    for(int i=0;i<4;i++){
        cv::Point2f ptIn (ptsIn [i*3+0], ptsIn [i*3+1]);
        cv::Point2f ptOut(ptsOut[i*3+0], ptsOut[i*3+1]);
        ptsInPt2f[i]  = ptIn+cv::Point2f(halfW,halfH);
        ptsOutPt2f[i] = (ptOut+cv::Point2f(1,1));

        ptsOutPt2f[i].x = ptsOutPt2f[i].x * (sideLengthX*0.5);
        ptsOutPt2f[i].y = ptsOutPt2f[i].y * (sideLengthY*0.5);
    }

    M=getPerspectiveTransform(ptsInPt2f,ptsOutPt2f);

    //Load corners vector
    if(corners){
        corners->clear();
        corners->push_back(ptsOutPt2f[0]);//Push Top Left corner
        corners->push_back(ptsOutPt2f[1]);//Push Top Right corner
        corners->push_back(ptsOutPt2f[2]);//Push Bottom Right corner
        corners->push_back(ptsOutPt2f[3]);//Push Bottom Left corner
    }
}

};
