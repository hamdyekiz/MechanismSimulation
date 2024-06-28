#include "FourBarMechanism.h"

void ParallelogramMechanism::checkGrashofTheorem()
{
    double thetaThreeAngle = 0.0, thetaFourAngle = 0.0;

    FourBarMechanism::angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle);

    std::cout << "The type of mechanism is Parallelogram Mechanism.\n";
    if (links[0] + links[1] == links[2] + links[3])
    {
        std::cout << "This mechanism is a special case.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }

    std::cout << "Theta Three Angle: " << thetaThreeAngle << " degrees" << std::endl;
    std::cout << "Theta Four Angle: " << thetaFourAngle << " degrees" << std::endl;
}