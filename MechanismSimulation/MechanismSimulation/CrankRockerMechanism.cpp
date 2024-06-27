#include "FourBarMechanism.h"

void CrankRockerMechanism::checkGrashofTheorem()
{
    int shortestLink, longestLink;
    double shortest, longest;
    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    std::cout << "The type of mechanism is Crank-Rocker Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - shortest - longest)
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }
}