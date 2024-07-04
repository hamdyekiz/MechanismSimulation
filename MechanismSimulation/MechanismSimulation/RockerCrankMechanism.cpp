#include "FourBarMechanism.h"
#include <fstream>
#include <iomanip>

const double PI = 3.14159265358979323846;


void RockerCrankMechanism::checkGrashofTheorem()
{
    int shortestLink, longestLink;
    double shortest, longest;
    double thetaThreeAngle = 0.0, thetaFourAngle = 0.0;

    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    std::cout << "The type of mechanism is Rocker-Crank Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }
}

std::pair <double, double> RockerCrankMechanism::rangeOfThetaTwoAngle(double links[4])
{
    double thetaTwoMin, thetaTwoMax;
    double thetaTwoMinRad = acos((pow(links[0], 2) + pow(links[3], 2) - pow(links[1] - links[2], 2)) / (2 * links[0] * links[3]));
    thetaTwoMin = radiansToDegrees(thetaTwoMinRad);
    thetaTwoMax = 360 - thetaTwoMin;

    return{ thetaTwoMin, thetaTwoMax };
}


void RockerCrankMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    // Create a text file named angles.txt
    std::ofstream outFile("angles.txt");
    if (!outFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }
    outFile << std::fixed << std::setprecision(10);

    std::pair<double, double> thetaTwoAngleRange = rangeOfThetaTwoAngle(links);
    double thetaTwoMin = thetaTwoAngleRange.first;
    double thetaTwoMax = thetaTwoAngleRange.second;

    for (thetaTwoAngle = thetaTwoMin; thetaTwoAngle <= thetaTwoMax; thetaTwoAngle += 0.1)
    {
        double thetaTwoAngleRad = degreesToRadians(thetaTwoAngle);
        double lengthL = lengthL_Calculator(links, thetaTwoAngleRad);
        double transmissionAngle = transmissionAngle_Calculator(links, lengthL);

        // Thus are used in calculating thetaThreeAngle
        double rLength = lengthR_Calculator(links, thetaTwoAngleRad);
        double sLength = lengthS_Calculator(links, thetaTwoAngleRad);
        double gLength = lengthG_Calculator(links, transmissionAngle);
        double hLength = lengthH_Calculator(links, transmissionAngle);

        thetaThreeAngle = thetaThreeAngle_Calculator(rLength, sLength, gLength, hLength);

        // Convert angles to degree 
        double thetaThreeAngleDeg = radiansToDegrees(thetaThreeAngle);
        double transmissionAngleDeg = radiansToDegrees(transmissionAngle);

        thetaFourAngle = thetaFourAngle_Calculator(thetaThreeAngleDeg, transmissionAngleDeg);


        outFile << thetaTwoAngle << " " << thetaThreeAngleDeg << " " << thetaFourAngle << std::endl;
    }
    std::cout << ("Simulation is finished");

    // Close the file when done
    outFile.close();
}

void RockerCrankMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{

}