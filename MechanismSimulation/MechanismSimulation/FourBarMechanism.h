#ifndef FOURBARMECHANISM_H
#define FOURBARMECHANISM_H
#include <iostream>

// Base class for Four-Bar Mechanisms
class FourBarMechanism {
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

    // Method to check the mechanism is obey the Grashof's Law or not
    virtual void checkGrashofTheorem(); 

    // Find the mechanism type with respect to location of the shortest link
    static FourBarMechanism* mechanismController(double links[4], double thetaTwoAngle);

public:
    // In this method user enters the Links of the mechanism
    static FourBarMechanism* userInput(); 
    /*
    This method find the shortest and longest link index and length
    shortestLink = index of the shortest link
    longestLink = index of the longest link
    shortest = length of the shortest link
    longest = length of the longest link
    */ 
    static void calculateShortestAndLongest(double links[4], int& shortestLink, int& longestLink, double& shortest, double& longest);
    static void angleFinder(double links[4], double thetaTwoAngle, double& thetaThreeAngle, double& thetaFourAngle);

};

// Crank-Rocker Mechanism class inherites from FourBarMechanism class (shortest link = crank) 
class CrankRockerMechanism : public FourBarMechanism {
public:
    CrankRockerMechanism(double Links[4], double ThetaTwoAngle)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        thetaTwoAngle = ThetaTwoAngle;
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Rocker-Crank Mechanism class inherites from FourBarMechanism class (shortest link = rocker) 
class RockerCrankMechanism : public FourBarMechanism {
public:
    RockerCrankMechanism(double Links[4], double ThetaTwoAngle)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        thetaTwoAngle = ThetaTwoAngle;
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Double-Crank Mechanism (Drag Link Mechanism) class inherites from FourBarMechanism class (shortest link = ground) 
class DoubleCrankMechanism : public FourBarMechanism {
public:
    DoubleCrankMechanism(double Links[4], double ThetaTwoAngle)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        thetaTwoAngle = ThetaTwoAngle;
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Double-Rocker Mechanism class inherites from FourBarMechanism class (shortest link = coupler)
class DoubleRockerMechanism : public FourBarMechanism {
public:
    DoubleRockerMechanism(double Links[4], double ThetaTwoAngle)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        thetaTwoAngle = ThetaTwoAngle;
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Parallelogram Mechanism inherites from FourBarMechanism class
class ParallelogramMechanism : public FourBarMechanism {
public:
    ParallelogramMechanism(double Links[4], double ThetaTwoAngle)
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        thetaTwoAngle = ThetaTwoAngle;
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

#endif // FOURBARMECHANISM_H

