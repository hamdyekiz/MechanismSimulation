#include "FourBarMechanism.h"

const double PI = 3.14159265358979323846;


void DoubleCrankMechanism::checkGrashofTheorem()
{
    int shortestLink, longestLink;
    double shortest, longest;
    double thetaThreeAngle = 0.0, thetaFourAngle = 0.0;

    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);
    FourBarMechanism::angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle);

    std::cout << "The type of mechanism is Double-Crank Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }

    std::cout << "Theta Three Angle: " << thetaThreeAngle << " degrees" << std::endl;
    std::cout << "Theta Four Angle: " << thetaFourAngle << " degrees" << std::endl;

}

void DoubleCrankMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    std::cout << "Please enter angle between link 1 and ground link (link 4): ";
    std::cin >> thetaTwoAngle;
}