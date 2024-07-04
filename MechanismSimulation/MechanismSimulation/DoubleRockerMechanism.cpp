#include "FourBarMechanism.h"
#include <fstream>
#include <vector>
#include <iomanip>

const double PI = 3.14159265358979323846;
const double TOLERANCE = 1e-2;
const double TOLERANCETHETAFOUR = 1e-6;

double calculateLengthL(double links[4], double phiAngle)
{
    phiAngle = phiAngle * (PI / 180.0); // Convert degrees to radian
    double lengthL = links[3] * cos(phiAngle) + sqrt(pow(links[0], 2) - pow(links[3], 2) * pow(sin(phiAngle), 2));
    return lengthL;
}

std::pair <double, double> rangeOfThetaTwoAngle(double links[4])
{
    double thetaTwoMin = acos((pow(links[0], 2) + pow(links[3], 2) - pow(links[2] - links[1], 2)) / 2 * links[0] * links[3]);
    double thetaTwoMax = acos((pow(links[0], 2) + pow(links[3], 2) - pow(links[2] + links[1], 2)) / 2 * links[0] * links[3]);
    return{ thetaTwoMin, thetaTwoMax };
}

std::pair <double, double> rangeOfThetaFourAngle(double links[4])
{
    double thetaFourMin = PI - acos((pow(links[2], 2) + pow(links[3], 2) - pow(links[0] + links[1], 2)) / 2 * links[2] * links[3]);
    double thetaFourMax = PI - acos((pow(links[2], 2) + pow(links[3], 2) - pow(links[0] - links[1], 2)) / 2 * links[2] * links[3]);
    return{ thetaFourMin, thetaFourMax };
}

std::pair <double, double> rangeOfLengthL(double links[4])
{
    double minL = links[2] - links[1];
    double maxL = links[2] + links[1];
    return { minL, maxL };
}

std::pair <double, double> rangeOfPhiAngle(double links[4], double minL, double maxL)
{
    double phiAngle;
    double phiMin, phiMax;
    double lengthL = 0;
    std::vector<double> phiAngles;

    for (lengthL = minL; lengthL <= maxL; lengthL += 1e-3)
    {
        phiAngle = acos((pow(lengthL, 2) + pow(links[3], 2) - pow(links[0], 2)) / (2 * lengthL * links[3]));
        phiAngles.push_back(phiAngle);
    }
    phiMin = *std::min_element(phiAngles.begin(), phiAngles.end());
    phiMax = *std::max_element(phiAngles.begin(), phiAngles.end());
    // Convert to degrees before return
    double phiMinDeg = phiMin * (180.0 / PI);
    double phiMaxDeg = phiMax * (180.0 / PI);

    return { phiMinDeg, phiMaxDeg };
}

double phiAngleFinder(double links[4], double thetaThreeAngle, double phiMin, double phiMax)
{
    double l1_pos, l1_neg, l2_pos, l2_neg; // l1 and l2 are both lengthL pos means positive root, neg means negative root
    double phiAngle;

    for (phiAngle = phiMin; phiAngle <= phiMax; phiAngle += 1e-2)
    {
        phiAngle = phiAngle * (PI / 180.0);

        l1_pos = links[3] * cos(phiAngle) + sqrt(pow(links[0], 2) - pow(links[3], 2) * pow(sin(phiAngle), 2));
        l1_neg = links[3] * cos(phiAngle) - sqrt(pow(links[0], 2) - pow(links[3], 2) * pow(sin(phiAngle), 2));

        l2_pos = links[1] * cos(phiAngle + thetaThreeAngle) + sqrt(pow(links[2], 2) - pow(links[1], 2) * pow(sin(phiAngle + thetaThreeAngle), 2));
        l2_neg = links[1] * cos(phiAngle + thetaThreeAngle) - sqrt(pow(links[2], 2) - pow(links[1], 2) * pow(sin(phiAngle + thetaThreeAngle), 2));

        // Check both l1 and l2 pairs are positive
        if (l1_pos >= 0 && l2_pos >= 0 && fabs(l1_pos - l2_pos) < TOLERANCE)
        {
            phiAngle = phiAngle * (180.0 / PI); // Convert to degrees before return
            return phiAngle;
        }
        if (l1_pos >= 0 && l2_neg >= 0 && fabs(l1_pos - l2_neg) < TOLERANCE)
        {
            phiAngle = phiAngle * (180.0 / PI); // Convert to degrees before return
            return phiAngle;
        }
        if (l1_neg >= 0 && l2_pos >= 0 && fabs(l1_neg - l2_pos) < TOLERANCE)
        {
            phiAngle = phiAngle * (180.0 / PI); // Convert to degrees before return
            return phiAngle;
        }
        if (l1_neg >= 0 && l2_neg >= 0 && fabs(l1_neg - l2_neg) < TOLERANCE)
        {
            phiAngle = phiAngle * (180.0 / PI); // Convert to degrees before return
            return phiAngle;
        }
        phiAngle = phiAngle * (180.0 / PI);
    }
}
double betaAngleFinder(double links[4], double phiAngle)
{
    double lengthL = calculateLengthL(links, phiAngle);

    double betaAngle = acos((pow(lengthL, 2) + pow(links[2], 2) - pow(links[1], 2)) / (2 * lengthL * links[2]));
    betaAngle = betaAngle * (180.0 / PI); // Convert to degree before return
    return betaAngle;
}


double thetaTwoAngleFinder(double links[4], double phiAngle)
{
    double lengthL = calculateLengthL(links, phiAngle);

    double thetaTwoAngle = acos((pow(links[0], 2) + pow(links[3], 2) - pow(lengthL, 2)) / (2 * links[0] * links[3]));
    thetaTwoAngle = thetaTwoAngle * (180.0 / PI); // TEMP
    return thetaTwoAngle;
}   
// THERE IS A PROBLEM HERE
double thetaFourAngleFinder(double links[4], double thetaThreeAngle, double phiAngle, double betaAngle)
{
    double lengthL = calculateLengthL(links, phiAngle);
    double thetaFourAngle;
    double phiAngleRad = phiAngle * (PI / 180);
    double betaAngleRad = betaAngle * (PI / 180);

    double controller = sqrt(pow(links[3], 2) + pow(lengthL, 2) - (2 * links[3] * lengthL * cos(phiAngleRad)));

    if (fabs(controller - links[0]) < TOLERANCETHETAFOUR)
    {
        thetaFourAngle = 180 - phiAngle + betaAngle;
    }
    else
    {
        thetaFourAngle = 180 - phiAngle - betaAngle;
    }

    return thetaFourAngle;
}



void DoubleRockerMechanism::checkGrashofTheorem()
{
    int shortestLink, longestLink;
    double shortest, longest;

    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    std::cout << "The type of mechanism is Double-Rocker Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }
}

void DoubleRockerMechanism::angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    // Create a text file named angles.txt
    std::ofstream outFile("angles.txt");
    if (!outFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }
    outFile << std::fixed << std::setprecision(10);

    std::pair<double, double> lengthLRange = rangeOfLengthL(links);
    double minL = lengthLRange.first;
    double maxL = lengthLRange.second;

    std::pair<double, double> phiAngleRange = rangeOfPhiAngle(links, minL, maxL);
    double phiMin = phiAngleRange.first;
    double phiMax = phiAngleRange.second;

    for (thetaThreeAngle = 0; thetaThreeAngle <= 360; thetaThreeAngle += 0.01)
    {
        double PhiAngle;
        double BetaAngle;

        double thetaThreeAngleRad = degreesToRadians(thetaThreeAngle); // Convert angle degrees to radians

        PhiAngle = phiAngleFinder(links, thetaThreeAngleRad, phiMin, phiMax);
        BetaAngle = betaAngleFinder(links, PhiAngle);
        thetaTwoAngle = thetaTwoAngleFinder(links, PhiAngle);
        thetaFourAngle = thetaFourAngleFinder(links, thetaThreeAngle, PhiAngle, BetaAngle);

        double thetaTwoAngleRad = degreesToRadians(thetaTwoAngle); // Convert angle degrees to radians
        double thetaFourAngleRad = degreesToRadians(thetaFourAngle); // Convert angle degrees to radians

        outFile << thetaTwoAngle << " " << thetaThreeAngle << " " << thetaFourAngle << " " << PhiAngle << " " << BetaAngle << std::endl;
        positionCalculator(links, thetaTwoAngleRad, thetaThreeAngleRad, thetaFourAngleRad);
        thetaThreeAngle = radiansToDegrees(thetaThreeAngleRad); // Convert angle radians to degree
    }
    std::cout << ("Simulation is finished");

    // Close the file when done
    outFile.close();
}
void DoubleRockerMechanism::positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle)
{
    // Create a text file named positions.txt
    std::ofstream outFile("positions.txt", std::ios::app);
    if (!outFile) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }

    // point A(xA, yA)
    double xA = links[0] * cos(thetaTwoAngle);
    double yA = links[0] * sin(thetaTwoAngle);

    // Write positions to the positions.txt file
    outFile << xA << " " << yA << std::endl;

    // Close the file when done
    outFile.close();
}