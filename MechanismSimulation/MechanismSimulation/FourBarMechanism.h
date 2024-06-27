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

    // Method to check the mechanism is obey the Grashof's Law or not
    virtual void checkGrashofTheorem(); 

    // Find the mechanism type with respect to location of the shortest link
    static FourBarMechanism* mechanismController(double links[4]); 

public:
    // In this method user enters the Links of the mechanism
    static FourBarMechanism* userLinkInput(); 
    // This method find the shortest and longest link
    static void calculateShortestAndLongest(double links[4], double& shortest, double& longest); 
};

// Crank-Rocker Mechanism class shortest link = crank inherites from FourBarMechanism class
class CrankRockerMechanism : public FourBarMechanism {
public:
    CrankRockerMechanism(double Links[4])
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Rocker-Crank Mechanism class shortest link = rocker inherites from FourBarMechanism class
class RockerCrankMechanism : public FourBarMechanism {
public:
    RockerCrankMechanism(double Links[4])
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Double-Crank Mechanism (Drag Link Mechanism) class shortest link = ground inherites from FourBarMechanism class
class DoubleCrankMechanism : public FourBarMechanism {
public:
    DoubleCrankMechanism(double Links[4])
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Double-Rocker Mechanism class shortest link = coupler inherites from FourBarMechanism class
class DoubleRockerMechanism : public FourBarMechanism {
public:
    DoubleRockerMechanism(double Links[4])
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

// Derived class for Parallelogram Mechanism inherites from FourBarMechanism class
class ParallelogramMechanism : public FourBarMechanism {
public:
    ParallelogramMechanism(double Links[4])
    {
        links[0] = Links[0];
        links[1] = Links[1];
        links[2] = Links[2];
        links[3] = Links[3];
        checkGrashofTheorem();
    }

    void checkGrashofTheorem() override;
};

#endif // FOURBARMECHANISM_H

