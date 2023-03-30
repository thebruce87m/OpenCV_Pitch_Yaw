#include <iostream>

#include "warper.h"


using namespace cv;
using namespace std;


int main(void){
    int c = 0;
    Mat m, disp;

    Warper warper;

    VideoCapture cap(0);

    while(c != 033 && cap.isOpened())
    {
        cap >> m;

        warper.warpImage(m, disp);

        imshow("Disp", disp);
        c = waitKey(1);

        switch (c)
        {
        case 'z':
        {
            auto theta = warper.GetTheta();

            warper.SetTheta(theta - 1);

            std::cout << "theta: " << warper.GetTheta() << std::endl;
            break;
        }
        
        case 'x':
        {
            auto theta = warper.GetTheta();

            warper.SetTheta(theta + 1);

            std::cout << "theta: " << warper.GetTheta() << std::endl;
            break;
        }

        case 'c':
        {
            auto phi = warper.GetPhi();

            warper.SetPhi(phi - 1);

            std::cout << "phi: " << warper.GetPhi() << std::endl;
            break;
        }
        
        case 'v':
        {
            auto phi = warper.GetPhi();

            warper.SetPhi(phi + 1);

            std::cout << "phi: " << warper.GetPhi() << std::endl;
            break;
        }

        case 'b':
        {
            auto gamma = warper.GetGamma();

            warper.SetGamma(gamma - 1);

            std::cout << "gamma: " << warper.GetGamma() << std::endl;
            break;
        }
        
        case 'n':
        {
            auto gamma = warper.GetGamma();

            warper.SetGamma(gamma + 1);

            std::cout << "gamma: " << warper.GetGamma() << std::endl;
            break;
        }

        case 'o':
        {
            auto fovy = warper.GetFovY();

            warper.SetFovY(fovy - 1);

            std::cout << "FOV Y: " << warper.GetFovY() << std::endl;
            break;
        }
        
        case 'p':
        {
            auto fovy = warper.GetFovY();

            warper.SetFovY(fovy + 1);

            std::cout << "FOV Y: " << warper.GetFovY() << std::endl;
            break;
        }

        case 'u':
        {
            auto scale = warper.GetScale();

            warper.SetScale(scale - 0.1);

            std::cout << "scale: " << warper.GetScale() << std::endl;
            break;
        }
        
        case 'i':
        {
            auto scale = warper.GetScale();

            warper.SetScale(scale + 0.1);

            std::cout << "scale: " << warper.GetScale() << std::endl;
            break;
        }


        case 'a':
        {
            auto xOffset = warper.GetXOffset();

            warper.SetXOffset(xOffset - 10);

            std::cout << "X Offset: " << warper.GetXOffset() << std::endl;
            break;
        }

        case 'd':
        {
            auto xOffset = warper.GetXOffset();

            warper.SetXOffset(xOffset + 10);

            std::cout << "X Offset: " << warper.GetXOffset() << std::endl;
            break;
        }

        case 's':
        {
            auto yOffset = warper.GetYOffset();

            warper.SetYOffset(yOffset + 10);

            std::cout << "Y Offset: " << warper.GetYOffset() << std::endl;
            break;
        }

        case 'w':
        {
            auto yOffset = warper.GetYOffset();

            warper.SetYOffset(yOffset - 10);

            std::cout << "Y Offset: " << warper.GetYOffset() << std::endl;
            break;
        }

        case 'e':
        {
            auto zOffset = warper.GetZOffset();

            warper.SetZOffset(zOffset + 10);

            std::cout << "Z Offset: " << warper.GetZOffset() << std::endl;
            break;
        }

        case 'r':
        {
            auto zOffset = warper.GetZOffset();

            warper.SetZOffset(zOffset - 10);

            std::cout << "Z Offset: " << warper.GetZOffset() << std::endl;
            break;
        }

        case -1:
            // Timeout
            break;

        default:
            std::cout << "Unknown Key: " << c << std::endl;
            break;
        }

    } // while
}
