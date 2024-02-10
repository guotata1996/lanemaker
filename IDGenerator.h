#pragma once

#include <map>
#include <string>
#include <vector>

class IDGenerator
{
public:
    IDGenerator(IDGenerator&) = delete;
    IDGenerator& operator=(const IDGenerator&) = delete;
    static IDGenerator* ForJunction();
    static IDGenerator* ForRoad();
    std::string GenerateID(void* object);
    void* GetByID(const std::string&);
    bool FreeID(const std::string&);

private:
    IDGenerator(std::string aType);

    static IDGenerator* _junction;
    static IDGenerator* _road;

    std::string type;
    std::vector<bool> assigned;
    std::map<uint32_t, void*> assignTo;
};

