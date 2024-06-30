#include "FourBarMechanism.h"
#include <fstream>


const double PI = 3.14159265358979323846;
const double TOLERANCE = 1e-2; 

double calculateLengthL(double links[4], double phiAngle)
{
    phiAngle = phiAngle * (PI / 180.0); // Convert degrees to radian
    double lengthL = links[3] * cos(phiAngle) + sqrt(pow(links[0], 2) - pow(links[3], 2) * pow(sin(phiAngle), 2));
    return lengthL;
}
// Continue from here
double phiAngleFinder(double links[4], double thetaThreeAngle)
{
    double l1_pos, l1_neg, l2_pos, l2_neg; // l1 and l2 are both lengthL pos means positive root, neg means negative root
    double phiAngle;

    for (phiAngle = 0; phiAngle <= 360; phiAngle += 0.01)
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
double thetaFourAngleFinder(double links[4], double thetaThreeAngle, double phiAngle, double betaAngle)
{
    double lengthL = calculateLengthL(links, phiAngle);
    double thetaFourAngle;
    double phiAngleRad = phiAngle * (PI / 180);
    double betaAngleRad = betaAngle * (PI / 180);

    double controller = sqrt(pow(links[3], 2) + pow(lengthL, 2) - 2 * links[3] * lengthL * cos(phiAngleRad + betaAngleRad));

    if (fabs(controller - links[0]) < TOLERANCE)
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

    for (thetaThreeAngle = 0; thetaThreeAngle <= 360; thetaThreeAngle += 0.1)
    {
        double PhiAngle;
        double BetaAngle;


        double thetaThreeAngleRad = degreesToRadians(thetaThreeAngle); // Convert angle degrees to radians

        PhiAngle = phiAngleFinder(links, thetaThreeAngleRad);
        BetaAngle = betaAngleFinder(links, PhiAngle);
        thetaTwoAngle = thetaTwoAngleFinder(links, PhiAngle);
        thetaFourAngle = thetaFourAngleFinder(links, thetaThreeAngle, PhiAngle, BetaAngle);

        double thetaTwoAngleRad = degreesToRadians(thetaTwoAngle); // Convert angle degrees to radians
        double thetaFourAngleRad = degreesToRadians(thetaFourAngle); // Convert angle degrees to radians

        outFile << thetaTwoAngle << " " << thetaThreeAngle << " " << thetaFourAngle << std::endl;
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