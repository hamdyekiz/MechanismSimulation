#include "FourBarMechanism.h"
#include <fstream>
#include <iomanip>
#include <vector>

const double PI = 3.14159265358979323846;


void DoubleRockerMechanism::checkGrashofTheorem(double shortest, double longest)
{
    std::cout << "The type of mechanism is Double-Rocker Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }
}

std::pair <double, double> DoubleRockerMechanism::rangeOfThetaTwoAngle(double links[4], double shortest, double longest)
{
    double thetaTwoMin, thetaTwoMax;
    double thetaTwoMaxRad = acos((pow(links[0], 2) + pow(links[3], 2) - pow(links[1] + links[2], 2)) / (2 * links[0] * links[3]));
    thetaTwoMax = radiansToDegrees(thetaTwoMaxRad);
    thetaTwoMin = -thetaTwoMax;

    return{ thetaTwoMin, thetaTwoMax };
}


void DoubleRockerMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest)
{
    // Check is there a position file before
    static bool isFileCleared = false;
    if (!isFileCleared) {
        std::ofstream positionFileClear("position.txt", std::ios::trunc);
        positionFileClear.close();
        isFileCleared = true;
    }

    // Create a text file named angles.txt
    std::ofstream angleFile("angles.txt");
    if (!angleFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }
    angleFile << std::fixed << std::setprecision(10);

    std::pair<double, double> thetaTwoAngleRange = rangeOfThetaTwoAngle(links, shortest, longest);
    double thetaTwoMin = thetaTwoAngleRange.first;
    double thetaTwoMax = thetaTwoAngleRange.second;

    for (thetaTwoAngle = thetaTwoMin; thetaTwoAngle <= thetaTwoMax; thetaTwoAngle += 0.1)
    {
        double thetaTwoAngleRad = degreesToRadians(thetaTwoAngle);
        double lengthL = lengthL_Calculator(links, thetaTwoAngleRad);
        double transmissionAngle = transmissionAngle_Calculator(links, lengthL);

        double rLength = lengthR_Calculator(links, thetaTwoAngleRad);
        double sLength = lengthS_Calculator(links, thetaTwoAngleRad);
        double gLength = lengthG_Calculator(links, transmissionAngle);
        double hLength = lengthH_Calculator(links, transmissionAngle);

        thetaThreeAngle = thetaThreeAngle_Calculator(rLength, sLength, gLength, hLength);

        // Convert angles to degree 
        double thetaThreeAngleDeg = radiansToDegrees(thetaThreeAngle);
        double transmissionAngleDeg = radiansToDegrees(transmissionAngle);

        thetaFourAngle = thetaFourAngle_Calculator(thetaThreeAngleDeg, transmissionAngleDeg);

        angleFile << thetaTwoAngle << " " << thetaThreeAngleDeg << " " << thetaFourAngle << std::endl;

        positionCalculator(links, thetaTwoAngleRad, thetaThreeAngle, degreesToRadians(thetaFourAngle));
    }
 
    std::cout << ("Simulation is finished");

    // Close the file when done
    angleFile.close();
}

void DoubleRockerMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    std::ofstream positionFile("position.txt", std::ios::app);
    if (!positionFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }
    positionFile << std::fixed << std::setprecision(10);

    std::vector<double> link1Position(2);
    link1Position[0] = links[0] * cos(thetaTwoAngle); // X position
    link1Position[1] = links[0] * sin(thetaTwoAngle); // Y position

    std::vector<double> link2Position(2);
    link2Position[0] = link1Position[0] + links[1] * cos(thetaThreeAngle); // X position
    link2Position[1] = link1Position[1] + links[1] * sin(thetaThreeAngle); // Y position

    std::vector<double> link3Position(2);
    link3Position[0] = links[3] + links[2] * cos(thetaFourAngle); // X position
    link3Position[1] = links[2] * sin(thetaFourAngle); // Y position

    // Fourth link is fixed
    std::vector<double> link4Position = { links[3], 0.0 }; // Ground link at origin

    positionFile << link1Position[0] << " " << link1Position[1] << " "
        << link2Position[0] << " " << link2Position[1] << " "
        << link3Position[0] << " " << link3Position[1] << " "
        << link4Position[0] << " " << link4Position[1] << std::endl;
}