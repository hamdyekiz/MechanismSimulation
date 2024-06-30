#include "FourBarMechanism.h"
#include <cmath>

const double PI = 3.14159265358979323846;

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

// Method to convert thetaTwoAngle to 0-360 range
double FourBarMechanism::normalizeAngle(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle < 0.0) {
        angle += 360.0;
    }
    return angle;
}

FourBarMechanism* FourBarMechanism::userLinkInput()
{
    double Links[4];
    double ThetaTwoAngle;

    std::cout << "Please enter length of the link 1: ";
    std::cin >> Links[0];

    std::cout << "Please enter length of the link 2: ";
    std::cin >> Links[1];

    std::cout << "Please enter length of the link 3: ";
    std::cin >> Links[2];

    std::cout << "Please enter length of the link 4: ";

    std::cin >> Links[3];

    //std::cout << "Please enter angle between link 1 and ground link (link 4): ";
    //std::cin >> ThetaTwoAngle;

    //ThetaTwoAngle = normalizeAngle(ThetaTwoAngle);

    return mechanismController(Links);
}

FourBarMechanism* FourBarMechanism::mechanismController(double links[4])
{
    int shortestLink, longestLink;
    double shortest, longest;
    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    if (links[0] == links[2] && links[1] == links[3])
    {
        return new ParallelogramMechanism(links);
    }
    else if (shortestLink == 3)
    {
        return new DoubleCrankMechanism(links);
    }
    else if (shortestLink == 2)
    {
        return new RockerCrankMechanism(links);
    }
    else if (shortestLink == 1)
    {
        return new DoubleRockerMechanism(links);
    }
    else if (shortestLink == 0)
    {
        return new CrankRockerMechanism(links);
    }
    else
    {
        std::cerr << "No suitable mechanism found for the given link lengths.\n";
        return nullptr;
    }
}

void FourBarMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    double lengthL;
    double alphaAngle;
    double PhiAngle;
    double BetaAngle;
 
    double thetaTwoAngleRad = degreesToRadians(thetaTwoAngle); // Convert degrees to radians

    lengthL = sqrt(pow(links[0], 2) + pow(links[3], 2) - 2 * links[0] * links[3] * cos(thetaTwoAngleRad));
    alphaAngle = acos((pow(lengthL, 2) + pow(links[1], 2) - pow(links[2], 2)) / (2 * lengthL * links[1]));
    PhiAngle = acos((pow(lengthL, 2) + pow(links[3], 2) - pow(links[0], 2)) / (2 * lengthL * links[3]));
    BetaAngle = acos((pow(lengthL, 2) + pow(links[2], 2) - pow(links[1], 2)) / (2 * lengthL * links[2]));

    alphaAngle = radiansToDegrees(alphaAngle);
    PhiAngle = radiansToDegrees(PhiAngle);
    BetaAngle = radiansToDegrees(BetaAngle);

    if (0 <= thetaTwoAngle && thetaTwoAngle <= 180)
    {
        thetaThreeAngle = -PhiAngle + alphaAngle;
        thetaFourAngle = 180 - PhiAngle - BetaAngle;
    }
    else if (180 < thetaTwoAngle && thetaTwoAngle <= 360)
    {
        thetaThreeAngle = PhiAngle + alphaAngle;
        thetaFourAngle = 180 + PhiAngle - BetaAngle;
    }
    else
    {
        std::cerr << "There might be problem.\n";
        return;
    }

}
void FourBarMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{

}
void FourBarMechanism::calculateShortestAndLongest(double links[4], int& shortestLink, int& longestLink, double& shortest, double& longest)
{
    shortest = links[0];
    longest = links[0];
    shortestLink = 0;
    longestLink = 0;

    for (int i = 1; i < 4; ++i) {
        if (links[i] < shortest) {
            shortest = links[i];
            shortestLink = i;
        }
        if (links[i] > longest) {
            longest = links[i];
            longestLink = i;
        }
    }
}
void FourBarMechanism::checkGrashofTheorem()
{
    std::cout << "Null ";
}





