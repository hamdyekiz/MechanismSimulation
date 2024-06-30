#include "FourBarMechanism.h"

const double PI = 3.14159265358979323846;



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
}

void ParallelogramMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    std::cout << "Please enter angle between link 1 and ground link (link 4): ";
    std::cin >> thetaTwoAngle;
}
void ParallelogramMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{

}