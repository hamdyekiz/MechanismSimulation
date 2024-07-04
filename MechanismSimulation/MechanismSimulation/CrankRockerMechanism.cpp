#include "FourBarMechanism.h"
#include <fstream>
#include <iomanip>

const double PI = 3.14159265358979323846;


void CrankRockerMechanism::checkGrashofTheorem()
{
    int shortestLink, longestLink;
    double shortest, longest;

    calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    std::cout << "The type of mechanism is Crank-Rocker Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }
}

std::pair <double, double> CrankRockerMechanism::rangeOfThetaTwoAngle(double links[4])
{
    int shortestLink, longestLink;
    double shortest, longest;
    double thetaTwoMin, thetaTwoMax;

    calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        thetaTwoMin = 0.0;
        thetaTwoMax = 360.0;
    }
    else
    {
        double thetaTwoMinRad = acos((pow(links[0], 2) + pow(links[3], 2) - pow(links[1] - links[2], 2)) / (2 * links[0] * links[3]));
        thetaTwoMin = radiansToDegrees(thetaTwoMinRad);
        thetaTwoMax = 360 - thetaTwoMin;
    }

    return{ thetaTwoMin, thetaTwoMax };
}

void CrankRockerMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
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
void CrankRockerMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{

}