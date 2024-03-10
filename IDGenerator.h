#pragma once

#include <map>
#include <string>
#include <vector>

// TODO: template? for type
class IDGenerator
{
    typedef uint32_t IDType;

public:
    IDGenerator(IDGenerator&) = delete;
    IDGenerator& operator=(const IDGenerator&) = delete;
    static IDGenerator* ForJunction();
    static IDGenerator* ForRoad();

    std::string GenerateID(void* object); // Added to changeList
    void ReuseID(void* object, const std::string& sid);
    void* GetByID(const std::string&);  // Added to changeList
    void NotifyChange(const std::string&);  // Added to changeList
    bool FreeID(const std::string&);  // Added to changeList

    std::map<IDType, void*> ConsumeChanges();
    
private:
    IDGenerator(std::string aType);

    static IDGenerator* _junction;
    static IDGenerator* _road;

    std::string type;
    std::vector<bool> assigned;
    std::map<IDType, void*> assignTo;

    std::map<IDType, void*> changeList; // Changes since last time ConsumeChanges() was called
};

