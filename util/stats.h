#pragma once

#include <string>
#include <map>

class Stats
{
public:
    Stats(std::string name);

    ~Stats();

    static Stats* Instance(std::string);

    void Increment(int delta = 1);

private:
    static std::map<std::string, Stats*> allInstances;

    static Stats* _dumpster;

    int count = 0;
    std::string name;
};