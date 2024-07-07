#include "FourBarMechanism.h"
#include <cmath>



const double PI = 3.14159265358979323846;

bool FourBarMechanism::isMechanism(double links[4])
{
    return (links[0] + links[1] + links[2] > links[3]) && (links[0] + links[1] + links[3] > links[2]) &&
        (links[0] + links[2] + links[3] > links[1]) && (links[1] + links[2] + links[3] > links[0]);
}

FourBarMechanism* FourBarMechanism::userLinkInput()
{
    double Links[4];
    double ThetaTwoAngle;
    while (true)
    {
        std::cout << "Please enter length of the link 1: ";
        std::cin >> Links[0];

        std::cout << "Please enter length of the link 2: ";
        std::cin >> Links[1];

        std::cout << "Please enter length of the link 3: ";
        std::cin >> Links[2];

        std::cout << "Please enter length of the link 4: ";

        std::cin >> Links[3];

        if (isMechanism(Links))
        {
            break; 
        }
        else
        {
            std::cout << "The link lengths do not form a valid mechanism. Please re-enter the lengths.\n";
        }
    }

    return mechanismController(Links);
}

FourBarMechanism* FourBarMechanism::mechanismController(double links[4])
{
    int shortestLink, longestLink;
    double shortest, longest;
    calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    if (links[0] == links[2] && links[1] == links[3])
    {
        return new ParallelogramMechanism(links, shortest, longest);
    }
    else if (shortestLink == 3)
    {
        return new DoubleCrankMechanism(links, shortest, longest);
    }
    else if (shortestLink == 2)
    {
        return new RockerCrankMechanism(links, shortest, longest);
    }
    else if (shortestLink == 1)
    {
        return new DoubleRockerMechanism(links, shortest, longest);
    }
    else if (shortestLink == 0)
    {
        return new CrankRockerMechanism(links, shortest, longest);
    }
    else
    {
        std::cerr << "No suitable mechanism found for the given link lengths.\n";
        return nullptr;
    }
}

void FourBarMechanism::calculateShortestAndLongest(double links[4], int& shortestLink, int& longestLink, double& shortest, double& longest)
{
    shortest = links[0];
    longest = links[0];
    shortestLink = 0;
    longestLink = 0;

    for (int i = 1; i < 4; ++i) {
        if (links[i] < shortest)
        {
            shortest = links[i];
            shortestLink = i;
        }
        if (links[i] > longest)
        {
            longest = links[i];
            longestLink = i;
        }
    }
}

// Method to convert degrees to radians
double FourBarMechanism::degreesToRadians(double degrees)
{
    return degrees * (PI / 180.0);
}

// Method to convert radians to degrees
double FourBarMechanism::radiansToDegrees(double radians)
{
    return radians * (180.0 / PI);
}

std::pair <double, double> FourBarMechanism::rangeOfThetaTwoAngle(double links[4], double shortest, double longest)
{

    double thetaTwoMin, thetaTwoMax;

    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        thetaTwoMin = 0.0;
        thetaTwoMax = 360.0;
    }
    else
    {
        double thetaTwoMinRad = acos((pow(links[0], 2) + pow(links[3], 2) - pow(links[1] + links[2], 2)) / (2 * links[0] * links[3]));
        thetaTwoMin = radiansToDegrees(thetaTwoMinRad);
        thetaTwoMax = 360 - thetaTwoMin;
    }

    return{ thetaTwoMin, thetaTwoMax };
}

double FourBarMechanism::lengthL_Calculator(double links[4], double thetaTwoAngle)
{
    double lengthL = sqrt(pow(links[0], 2) + pow(links[3], 2) - (2 * links[0] * links[3] * cos(thetaTwoAngle)));
    return lengthL;
}

double FourBarMechanism::transmissionAngle_Calculator(double links[4], double lengthL)
{
    // transmissionAngle is angle between link 2 and link 3
    double transmissionAngle = acos((pow(links[1], 2) + pow(links[2], 2) - pow(lengthL, 2)) / (2 * links[1] * links[2]));
    return transmissionAngle;
}

double FourBarMechanism::lengthR_Calculator(double links[4], double thetaTwoAngle)
{
    double rLength = links[3] - links[0] * cos(thetaTwoAngle);
    return rLength;
}

double FourBarMechanism::lengthS_Calculator(double links[4], double thetaTwoAngle)
{
    double sLength = links[0] * sin(thetaTwoAngle);
    return sLength;
}

double FourBarMechanism::lengthG_Calculator(double links[4], double transmissionAngle)
{
    double gLength = links[1] - links[2] * cos(transmissionAngle);
    return gLength;
}

double FourBarMechanism::lengthH_Calculator(double links[4], double transmissionAngle)
{
    double hLength = links[2] * sin(transmissionAngle);
    return hLength;
}

double FourBarMechanism::thetaThreeAngle_Calculator(double rLength, double sLength, double gLength, double hLength)
{
    double thetaThreeAngle = atan2((hLength * rLength - gLength * sLength), (gLength * rLength + hLength * sLength));
    return thetaThreeAngle;
}

double FourBarMechanism::thetaFourAngle_Calculator(double thetaThreeAngle, double transmissionAngle)
{
    double thetaFourAngle = transmissionAngle + thetaThreeAngle;
    return thetaFourAngle;
}

void FourBarMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest)
{
    std::cerr << "There might be a problem.\n";
}
void FourBarMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    std::cerr << "There might be a problem.\n";
}

void FourBarMechanism::checkGrashofTheorem(double shortest, double longest)
{
    std::cerr << "There might be a problem.\n";
}





