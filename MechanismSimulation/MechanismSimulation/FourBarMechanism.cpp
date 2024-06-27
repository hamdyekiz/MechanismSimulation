#include "FourBarMechanism.h"

FourBarMechanism* FourBarMechanism::userLinkInput()
{
    double Links[4];

    std::cout << "Please enter length of the link 1: ";
    std::cin >> Links[0];

    std::cout << "Please enter length of the link 2: ";
    std::cin >> Links[1];

    std::cout << "Please enter length of the link 3: ";
    std::cin >> Links[2];

    std::cout << "Please enter length of the link 4: ";
    std::cin >> Links[3];

    return mechanismController(Links);
}

FourBarMechanism* FourBarMechanism::mechanismController(double links[4])
{
    int shortestLink, longestLink;
    double shortest, longest;
    FourBarMechanism::calculateShortestAndLongest(links, shortestLink, longestLink, shortest, longest);

    if (links[0] == links[2] && links[1] == links[3])
    {
        return new ParallelogramMechanism(links);
    }
    else if (shortestLink == 3)
    {
        return new DoubleCrankMechanism(links);
    }
    else if (shortestLink == 2)
    {
        return new RockerCrankMechanism(links);
    }
    else if (shortestLink == 1)
    {
        return new DoubleRockerMechanism(links);
    }
    else if (shortestLink == 0)
    {
        return new CrankRockerMechanism(links);
    }
    else
    {
        std::cerr << "No suitable mechanism found for the given link lengths.\n";
        return nullptr;
    }
}

void FourBarMechanism::calculateShortestAndLongest(double links[4], int& shortestLink, int& longestLink, double& shortest, double& longest)
{
    shortest = links[0];
    longest = links[0];
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
void FourBarMechanism::checkGrashofTheorem()
{
    std::cout << "Null ";
}





