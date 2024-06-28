#include "FourBarMechanism.h"

void CrankRockerMechanism::checkGrashofTheorem()
{
    int shortestLink, longestLink;
    double shortest, longest;
    double thetaThreeAngle = 0.0, thetaFourAngle = 0.0;

    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);
    FourBarMechanism::angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle);

    std::cout << "The type of mechanism is Crank-Rocker Mechanism.\n";
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