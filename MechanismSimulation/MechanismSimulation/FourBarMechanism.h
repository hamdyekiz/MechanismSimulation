#ifndef FOURBARMECHANISM_H
#define FOURBARMECHANISM_H
#include <iostream>

// Base class for Four-Bar Mechanisms
class FourBarMechanism 
{
    public:
    // In this method user enters the Links of the mechanism
    static FourBarMechanism* userLinkInput();
private:
   static bool isMechanism(double links[4]);

protected:
    /* Lengths of the four-bar mechanism
    links[0] = Link 1 (crank/driver)
    links[1] = Link 2 (coupler/connecting rod)
    links[2] = Link 3 (rocker/lever/follower)
    links[3] = Link 4 (ground/frame)
    */
    double links[4];
    // thetaTwoAngle = Angle between link 1 and ground link (link 4)
    double thetaTwoAngle;
    double thetaThreeAngle;
    double thetaFourAngle;

    int shortestLink, longestLink;
    double shortest, longest;

    // Method to check the mechanism is obey the Grashof's Law or not
    virtual void checkGrashofTheorem(double shortest, double longest);

    virtual void angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest);
    virtual void positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle);

    // Find the mechanism type with respect to location of the shortest link
    static FourBarMechanism* mechanismController(double Links[4]);

    virtual std::pair <double, double> rangeOfThetaTwoAngle(double links[4], double shortest, double longest);

    // Method to convert degrees to radians
    static double degreesToRadians(double degrees);
    // Method to convert radians to degrees
    static double radiansToDegrees(double radians);

    double transmissionAngle_Calculator(double links[4], double lengthL);
    double thetaThreeAngle_Calculator(double rLength, double sLength, double gLength, double hLength);
    double thetaFourAngle_Calculator(double thetaThreeAngle, double transmissionAngle);

    double lengthL_Calculator(double links[4], double thetaTwoAngle);
    double lengthG_Calculator(double links[4], double transmissionAngle);
    double lengthH_Calculator(double links[4], double transmissionAngle);
    double lengthR_Calculator(double links[4], double thetaTwoAngle);
    double lengthS_Calculator(double links[4], double thetaTwoAngle);
    /*
    This method find the shortest and longest link index and length
    shortestLink = index of the shortest link
    longestLink = index of the longest link
    shortest = length of the shortest link
    longest = length of the longest link
    */ 
    static void calculateShortestAndLongest(double links[4], int& shortestLink, int& longestLink, double& shortest, double& longest);

};

// Crank-Rocker Mechanism class inherites from FourBarMechanism class (shortest link = crank) 
class CrankRockerMechanism : public FourBarMechanism 
{
public:
    CrankRockerMechanism(double Links[4], double Shortest, double Longest)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        shortest = Shortest;
        longest = Longest;

        checkGrashofTheorem(shortest, longest);
        angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle, shortest, longest);
    }

    void checkGrashofTheorem(double shortest, double longest) override;
    void angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest) override;
    void positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle) override;
    std::pair <double, double> rangeOfThetaTwoAngle(double links[4], double shortest, double longest) override;
};

// Rocker-Crank Mechanism class inherites from FourBarMechanism class (shortest link = rocker) 
class RockerCrankMechanism : public FourBarMechanism 
{
public:
    RockerCrankMechanism(double Links[4], double Shortest, double Longest)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        shortest = Shortest;
        longest = Longest;

        checkGrashofTheorem(shortest, longest);
        angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle, shortest, longest);
    }

    void checkGrashofTheorem(double shortest, double longest) override;
    void angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest) override;
    void positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle) override;
    std::pair <double, double> rangeOfThetaTwoAngle(double links[4], double shortest, double longest) override;
};

// Double-Crank Mechanism (Drag Link Mechanism) class inherites from FourBarMechanism class (shortest link = ground) 
class DoubleCrankMechanism : public FourBarMechanism 
{
public:
    DoubleCrankMechanism(double Links[4], double Shortest, double Longest)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        shortest = Shortest;
        longest = Longest;

        checkGrashofTheorem(shortest, longest);
        angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle, shortest, longest);
    }

    void checkGrashofTheorem(double shortest, double longest) override;
    void angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest) override;
    void positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle) override;
    std::pair <double, double> rangeOfThetaTwoAngle(double links[4], double shortest, double longest) override;
};

// Double-Rocker Mechanism class inherites from FourBarMechanism class (shortest link = coupler)
class DoubleRockerMechanism : public FourBarMechanism 
{
public:
    DoubleRockerMechanism(double Links[4], double Shortest, double Longest)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        shortest = Shortest;
        longest = Longest;

        checkGrashofTheorem(shortest, longest);
        angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle, shortest, longest);
    }

    void checkGrashofTheorem(double shortest, double longest) override;
    void angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest) override;
    void positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle) override;
    std::pair <double, double> rangeOfThetaTwoAngle(double links[4], double shortest, double longest) override;
};

// Parallelogram Mechanism inherites from FourBarMechanism class
class ParallelogramMechanism : public FourBarMechanism {
public:
    ParallelogramMechanism(double Links[4], double Shortest, double Longest)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        shortest = Shortest;
        longest = Longest;

        checkGrashofTheorem(shortest, longest);
        angleFinder(links, thetaTwoAngle, thetaThreeAngle, thetaFourAngle, shortest, longest);
    }

    void checkGrashofTheorem(double shortest, double longest) override;
    void angleFinder(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle, double shortest, double longest) override;
    void positionCalculator(double links[4], double thetaTwoAngle, double thetaThreeAngle, double thetaFourAngle) override;
    std::pair <double, double> rangeOfThetaTwoAngle(double links[4], double shortest, double longest) override;
};

#endif // FOURBARMECHANISM_H

