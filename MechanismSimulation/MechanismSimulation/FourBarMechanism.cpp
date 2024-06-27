#include "FourBarMechanism.h"

void FourBarMechanism::checkGrashofTheorem()
{
    std::cout << "Null ";
}

FourBarMechanism* FourBarMechanism::mechanismController(double links[4])
{
    double shortest, longest;
    FourBarMechanism::calculateShortestAndLongest(links, shortest, longest);

    if (links[0] == links[2] && links[1] == links[3])
    {
        return new ParallelogramMechanism(links);
    }
    else if (shortest == 3)
    {
        return new DoubleCrankMechanism(links);
    }
    else if (shortest == 2)
    {
        return new RockerCrankMechanism(links);
    }
    else if (shortest == 1)
    {
        return new DoubleRockerMechanism(links);
    }
    else if (shortest == 0)
    {
        return new CrankRockerMechanism(links);
    }
    else
    {
        // You can add a default mechanism if needed, or handle the default case
        std::cerr << "No suitable mechanism found for the given link lengths.\n";
        return nullptr;
    }
}

FourBarMechanism* FourBarMechanism::userLinkInput()
{
    double Links[4];

    std::cout << "Please enter link 1: ";
    std::cin >> Links[0];

    std::cout << "Please enter link 2: ";
    std::cin >> Links[1];

    std::cout << "Please enter link 3: ";
    std::cin >> Links[2];

    std::cout << "Please enter link 4: ";
    std::cin >> Links[3];

    return mechanismController(Links);
}

void FourBarMechanism::calculateShortestAndLongest(double links[4], double& shortestLink, double& longestLink)
{
    int shortest = links[0];
    int longest = links[0];
    shortestLink = 0;
    longestLink = 0;

    for (int i = 1; i < 4; ++i) {
        if (links[i] < shortest) {
            shortest = links[i];
            shortestLink = i;
        }
        if (links[i] > longest) {
            longest = links[i];
            longestLink = i;
        }
    }
}





