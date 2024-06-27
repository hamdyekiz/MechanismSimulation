#include "FourBarMechanism.h"

void DoubleRockerMechanism::checkGrashofTheorem()
{
    double shortest, longest;
    FourBarMechanism::calculateShortestAndLongest(links, shortest, longest);

    std::cout << "The type of mechanism is Double-Rocker Mechanism.\n";
    if (shortest + longest <= links[0] + links[1] + links[2] + links[3] - (shortest + longest))
    {
        std::cout << "This mechanism satisfies Grashof's theorem.\n";
    }
    else {
        std::cout << "This mechanism does not satisfy Grashof's theorem.\n";
    }
}